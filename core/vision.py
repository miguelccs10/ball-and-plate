#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Núcleo de Percepção Visual e Geometria da Placa (core/vision.py)
Captura V4L2, Transformação 2D Invertível e Detectores Visuais (HSV e YOLO)
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import os
import time
import threading
from typing import Tuple, Optional

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import numpy as np
except ImportError:
    np = None


# ==============================================================================
# 1. CAPTURA DE VÍDEO DE BAIXA LATÊNCIA (V4L2 / USB)
# ==============================================================================
class CameraStream:
    """Captura assíncrona em thread dedicada com descarte de frames em buffer."""

    def __init__(self, src=0, width=320, height=240, fps=30):
        self.src = src
        self.width = width
        self.height = height
        self.fps = fps

        self.cap = None
        self.frame = None
        self.timestamp = 0.0
        self.ret = False
        self.frame_id = 0
        self.running = False
        self.lock = threading.Lock()
        self.new_frame = threading.Condition(self.lock)
        self.thread = None

    def start(self):
        if cv2 is None:
            raise RuntimeError("OpenCV (cv2) não está disponível no ambiente.")

        self.cap = cv2.VideoCapture(self.src, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.src)

        if not self.cap.isOpened():
            raise RuntimeError(f"Erro: Não foi possível abrir a câmera no índice {self.src}.")

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.ret, self.frame = self.cap.read()
        self.timestamp = time.time()

        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, name="CameraCaptureThread", daemon=True)
        self.thread.start()
        return self

    def _capture_loop(self):
        while self.running and self.cap.isOpened():
            ret, frame = self.cap.read()
            now = time.time()
            if ret:
                with self.new_frame:
                    self.ret = ret
                    self.frame = frame
                    self.timestamp = now
                    self.frame_id += 1
                    self.new_frame.notify_all()
            else:
                time.sleep(0.005)

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None, 0.0
            return self.ret, self.frame, self.timestamp

    def wait_for_frame(self, last_id: int, timeout: float = 0.2):
        with self.new_frame:
            if self.frame_id <= last_id:
                self.new_frame.wait_for(lambda: self.frame_id > last_id or not self.running, timeout)
            if self.frame is None or self.frame_id <= last_id:
                return None, 0.0, last_id
            return self.frame, self.timestamp, self.frame_id

    def stop(self):
        self.running = False
        with self.new_frame:
            self.new_frame.notify_all()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.cap and self.cap.isOpened():
            self.cap.release()

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


# ==============================================================================
# 2. GEOMETRIA 2D BIJETIVA (PIXEL <-> PLACA MM)
# ==============================================================================
def roi_scale_mm_per_px(roi_w: int, roi_h: int, plate_diameter_mm: float) -> float:
    """Escala mm/px assumindo que a menor dimensão da ROI cobre o diâmetro da placa."""
    return plate_diameter_mm / float(max(1, min(roi_w, roi_h)))


def pixel_to_plate_mm(px_x: float, px_y: float, roi_w: int, roi_h: int, scale: float,
                      rotation_deg: int = 0, invert_x: bool = False,
                      invert_y: bool = False, swap_xy: bool = False) -> Tuple[float, float]:
    """Converte posição em pixels na ROI para mm com origem no centro da placa."""
    dx = (px_x - roi_w / 2.0) * scale
    dy = -(px_y - roi_h / 2.0) * scale

    if rotation_deg == 90:
        rx, ry = -dy, dx
    elif rotation_deg == 180:
        rx, ry = -dx, -dy
    elif rotation_deg == 270:
        rx, ry = dy, -dx
    else:
        rx, ry = dx, dy

    if swap_xy:
        rx, ry = ry, rx
    if invert_x:
        rx = -rx
    if invert_y:
        ry = -ry
    return rx, ry


def plate_mm_to_pixel(rx_mm: float, ry_mm: float, roi_w: int, roi_h: int, scale: float,
                      rotation_deg: int = 0, invert_x: bool = False,
                      invert_y: bool = False, swap_xy: bool = False) -> Tuple[float, float]:
    """Inversa exata: converte coordenadas físicas (mm) de volta para pixels na ROI."""
    rx = -rx_mm if invert_x else rx_mm
    ry = -ry_mm if invert_y else ry_mm

    if swap_xy:
        rx, ry = ry, rx

    if rotation_deg == 90:
        dx, dy = ry, -rx
    elif rotation_deg == 180:
        dx, dy = -rx, -ry
    elif rotation_deg == 270:
        dx, dy = -ry, rx
    else:
        dx, dy = rx, ry

    safe_scale = max(1e-9, float(scale))
    px_x = (dx / safe_scale) + (roi_w / 2.0)
    px_y = (roi_h / 2.0) - (dy / safe_scale)
    return px_x, px_y


# ==============================================================================
# 3. DETECTOR VISUAL HSV
# ==============================================================================
class HSVBallDetector:
    """Detector baseado em limiarização HSV com cache de tuplas e momentos espaciais."""

    def __init__(self, hsv_lower=(5, 120, 100), hsv_upper=(25, 255, 255),
                 min_radius=4, max_radius=45, blur_size=5):
        self._lower_tuple = tuple(int(x) for x in hsv_lower)
        self._upper_tuple = tuple(int(x) for x in hsv_upper)
        self.hsv_lower = np.array(self._lower_tuple, dtype=np.uint8) if np is not None else list(self._lower_tuple)
        self.hsv_upper = np.array(self._upper_tuple, dtype=np.uint8) if np is not None else list(self._upper_tuple)
        self.min_radius = int(min_radius)
        self.max_radius = int(max_radius)
        self.blur_size = int(blur_size) if blur_size % 2 == 1 else int(blur_size + 1)

        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)) if cv2 is not None else None
        self.latest_mask = None

    def get_name(self) -> str:
        return "HSV"

    def set_parameters(self, lower=None, upper=None, blur=None, min_radius=None, max_radius=None):
        if lower is not None:
            lt = tuple(int(x) for x in lower)
            if lt != self._lower_tuple:
                self._lower_tuple = lt
                self.hsv_lower = np.array(lt, dtype=np.uint8) if np is not None else list(lt)
        if upper is not None:
            ut = tuple(int(x) for x in upper)
            if ut != self._upper_tuple:
                self._upper_tuple = ut
                self.hsv_upper = np.array(ut, dtype=np.uint8) if np is not None else list(ut)
        if blur is not None:
            b = int(blur)
            self.blur_size = b if b % 2 == 1 else b + 1
        if min_radius is not None:
            self.min_radius = max(1, int(min_radius))
        if max_radius is not None:
            self.max_radius = max(self.min_radius + 1, int(max_radius))

    def detect(self, frame) -> Tuple[bool, Tuple[float, float, float], float, Optional[Tuple[int, int, int, int]]]:
        if frame is None or cv2 is None or np is None:
            return False, (0.0, 0.0, 0.0), 0.0, None

        blurred = cv2.GaussianBlur(frame, (self.blur_size, self.blur_size), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        if self.kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)

        self.latest_mask = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, (0.0, 0.0, 0.0), 0.0, None

        best_contour = None
        best_area = 0.0
        best_radius = 0.0
        best_center = (0.0, 0.0)

        for c in contours:
            area = cv2.contourArea(c)
            if area < (np.pi * (self.min_radius ** 2) * 0.4):
                continue
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if self.min_radius <= radius <= self.max_radius:
                if area > best_area:
                    best_area = area
                    best_contour = c
                    best_radius = radius
                    best_center = (x, y)

        if best_contour is None:
            return False, (0.0, 0.0, 0.0), 0.0, None

        M = cv2.moments(best_contour)
        if M["m00"] > 1e-5:
            cx = float(M["m10"] / M["m00"])
            cy = float(M["m01"] / M["m00"])
        else:
            cx, cy = float(best_center[0]), float(best_center[1])

        x, y, w, h = cv2.boundingRect(best_contour)
        circle_area = np.pi * (best_radius ** 2)
        confidence = min(1.0, max(0.1, best_area / max(1.0, circle_area)))

        return True, (cx, cy, float(best_radius)), float(confidence), (x, y, w, h)


# ==============================================================================
# 4. DETECTOR VISUAL NEURAL YOLO
# ==============================================================================
class YOLOBallDetector:
    """Detector neural leve baseado em OpenCV DNN com pós-processamento vetorizado."""

    def __init__(self, model_path=None, conf_threshold=0.45, nms_threshold=0.40,
                 input_size=(224, 224), target_class_id: Optional[int] = 32):
        self.model_path = model_path
        self.conf_threshold = float(conf_threshold)
        self.nms_threshold = float(nms_threshold)
        self.input_size = tuple(input_size)
        self.target_class_id = target_class_id
        self.net = None
        self.is_loaded = False

        if model_path and os.path.exists(model_path):
            self.load_model(model_path)

    def get_name(self) -> str:
        return "YOLO"

    def load_model(self, model_path: str):
        if cv2 is None:
            return
        try:
            self.net = cv2.dnn.readNetFromONNX(model_path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self.is_loaded = True
            print(f"[YOLO] Modelo neural carregado: {model_path}")
        except Exception as e:
            print(f"[YOLO] Falha ao carregar ONNX ({e}).")
            self.is_loaded = False

    def detect(self, frame) -> Tuple[bool, Tuple[float, float, float], float, Optional[Tuple[int, int, int, int]]]:
        if not self.is_loaded or self.net is None or frame is None or cv2 is None or np is None:
            return False, (0.0, 0.0, 0.0), 0.0, None

        h_orig, w_orig = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            frame, scalefactor=1.0 / 255.0, size=self.input_size,
            mean=(0, 0, 0), swapRB=True, crop=False
        )
        self.net.setInput(blob)
        outputs = self.net.forward()

        if outputs.ndim != 3:
            return False, (0.0, 0.0, 0.0), 0.0, None

        output = outputs[0]
        if output.shape[0] < output.shape[1]:
            output = output.T

        num_cols = output.shape[1]
        if num_cols <= 4:
            return False, (0.0, 0.0, 0.0), 0.0, None

        num_classes = num_cols - 4
        if num_classes == 1:
            scores = output[:, 4]
        elif self.target_class_id is not None and 0 <= self.target_class_id < num_classes:
            scores = output[:, 4 + self.target_class_id]
        else:
            scores = np.max(output[:, 4:], axis=1)

        mask = scores >= self.conf_threshold
        if not np.any(mask):
            return False, (0.0, 0.0, 0.0), 0.0, None

        filtered_boxes = output[mask, :4]
        filtered_scores = scores[mask]

        scale_x = w_orig / float(self.input_size[0])
        scale_y = h_orig / float(self.input_size[1])

        cx = filtered_boxes[:, 0]
        cy = filtered_boxes[:, 1]
        w = filtered_boxes[:, 2]
        h = filtered_boxes[:, 3]

        x = ((cx - 0.5 * w) * scale_x).astype(np.int32)
        y = ((cy - 0.5 * h) * scale_y).astype(np.int32)
        w_px = (w * scale_x).astype(np.int32)
        h_px = (h * scale_y).astype(np.int32)

        boxes_list = np.stack([x, y, w_px, h_px], axis=1).tolist()
        conf_list = filtered_scores.astype(float).tolist()

        indices = cv2.dnn.NMSBoxes(boxes_list, conf_list, self.conf_threshold, self.nms_threshold)
        if len(indices) > 0:
            idx = int(np.asarray(indices).flat[0])
            bx, by, bw, bh = boxes_list[idx]
            best_conf = float(conf_list[idx])
            center_x = bx + (bw / 2.0)
            center_y = by + (bh / 2.0)
            est_radius = (bw + bh) / 4.0
            return True, (float(center_x), float(center_y), float(est_radius)), best_conf, (bx, by, bw, bh)

        return False, (0.0, 0.0, 0.0), 0.0, None


# ==============================================================================
# 5. FÁBRICA DE DETECTORES
# ==============================================================================
def create_detector(detector_type: str, **kwargs):
    d_type = detector_type.lower().strip()
    if d_type in ["hsv", "color"]:
        return HSVBallDetector(
            hsv_lower=kwargs.get("hsv_lower", (5, 120, 100)),
            hsv_upper=kwargs.get("hsv_upper", (25, 255, 255)),
            min_radius=kwargs.get("min_radius", 4),
            max_radius=kwargs.get("max_radius", 45),
            blur_size=kwargs.get("blur_size", 5)
        )
    elif d_type == "yolo":
        return YOLOBallDetector(
            model_path=kwargs.get("model_path"),
            conf_threshold=kwargs.get("conf_threshold", 0.45),
            nms_threshold=kwargs.get("nms_threshold", 0.40),
            input_size=kwargs.get("input_size", (224, 224)),
            target_class_id=kwargs.get("target_class_id", 32)
        )
    else:
        raise ValueError(f"Tipo de detector desconhecido: '{detector_type}'. Use 'hsv' ou 'yolo'.")
