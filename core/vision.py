#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Núcleo de Percepção Visual e Geometria da Placa (core/vision.py)
Captura V4L2 com descarte de buffer, Calibração Homográfica AprilTag (ArUco 36h11),
Compensação Geométrica de Paralaxe e Detector Visual HSV Otimizado.
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import os
import time
import math
import threading
from typing import Tuple, Optional, Dict, List, Any

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import numpy as np
except ImportError:
    np = None

try:
    import config
except ImportError:
    config = None


# ==============================================================================
# 1. CAPTURA DE VÍDEO DE BAIXA LATÊNCIA (V4L2 / USB)
# ==============================================================================
class CameraStream:
    """Captura assíncrona em thread dedicada com descarte de frames em buffer."""

    def __init__(self, camera_index: int = 0, width: int = 320, height: int = 240,
                 fps: int = 30, src: Optional[Any] = None):
        self.src = src if src is not None else camera_index
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)

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

    def read(self) -> Tuple[bool, Optional[Any], float]:
        with self.lock:
            if self.frame is None:
                return False, None, 0.0
            return self.ret, self.frame, self.timestamp

    def wait_for_frame(self, last_id: int, timeout: float = 0.2) -> Tuple[Optional[Any], float, int]:
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
# 2. GEOMETRIA 2D BIJETIVA E COMPENSAÇÃO DE PARALAXE
# ==============================================================================
def apply_parallax_correction(x_mm: float, y_mm: float,
                              cam_height_mm: float,
                              z_ball: float) -> Tuple[float, float]:
    """
    Aplica correção geométrica de paralaxe para objetos com elevação z_ball acima da placa.

    Fórmula:
        factor = max(0.75, min(1.25, (cam_height_mm - z_ball) / cam_height_mm))
        x_corr = x_mm * factor
        y_corr = y_mm * factor
    """
    if cam_height_mm is None or float(cam_height_mm) <= 1e-6:
        return float(x_mm), float(y_mm)
    z = float(z_ball) if z_ball is not None else 0.0
    h = float(cam_height_mm)
    ratio = (h - z) / h
    factor = max(0.75, min(1.25, ratio))
    return float(x_mm) * factor, float(y_mm) * factor


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
# 3. CALIBRADOR HOMOGRÁFICO PLANAR APRILTAG (ArUco 36h11)
# ==============================================================================
class AprilTagCalibrator:
    """
    Calibrador homográfico planar 2D baseado em marcadores AprilTag (DICT_APRILTAG_36h11).
    Utiliza OpenCV native cv2.aruco.ArucoDetector com refinamento subpixel e RANSAC.
    """

    def __init__(self,
                 dict_name: str = "DICT_APRILTAG_36h11",
                 tag_size_mm: float = 30.0,
                 tag_layout_mm: Optional[Dict[int, Tuple[float, float]]] = None,
                 cam_height_mm: float = 450.0,
                 ball_radius_mm: float = 20.0):
        self.tag_size_mm = float(tag_size_mm)
        self.cam_height_mm = float(cam_height_mm)
        self.ball_radius_mm = float(ball_radius_mm)

        cfg_layout = getattr(config, "APRILTAG_LAYOUT_MM", None) if config is not None else None
        if tag_layout_mm is not None:
            self.tag_layout_mm = {int(k): (float(v[0]), float(v[1])) for k, v in tag_layout_mm.items()}
        elif cfg_layout is not None:
            self.tag_layout_mm = {int(k): (float(v[0]), float(v[1])) for k, v in cfg_layout.items()}
        else:
            self.tag_layout_mm = {
                0: (-110.0, 110.0),
                1: (110.0, 110.0),
                2: (110.0, -110.0),
                3: (-110.0, -110.0),
            }

        self.is_calibrated: bool = False
        self.tags_detected: int = 0
        self.estimated_cam_tilt_deg: float = 0.0
        self.last_corners: list = []
        self.last_ids: list = []
        self.H: Optional[np.ndarray] = None
        self.H_inv: Optional[np.ndarray] = None

        self.dictionary = None
        self.params = None
        self.detector = None

        if cv2 is not None and hasattr(cv2, "aruco"):
            dict_id = getattr(cv2.aruco, dict_name, getattr(cv2.aruco, "DICT_APRILTAG_36h11", 20))
            self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
            self.params = cv2.aruco.DetectorParameters()
            if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
                self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)

    def reset(self):
        """Reseta todos os parâmetros e matrizes de calibração."""
        self.is_calibrated = False
        self.tags_detected = 0
        self.estimated_cam_tilt_deg = 0.0
        self.last_corners = []
        self.last_ids = []
        self.H = None
        self.H_inv = None

    def calibrate_from_frame(self, frame: np.ndarray) -> bool:
        """
        Detecta AprilTags no frame, refina subpixels, estima homografia via RANSAC
        e calcula o ângulo de inclinação da câmera. Retorna True em caso de sucesso.
        """
        if frame is None or self.detector is None or cv2 is None or np is None:
            self.tags_detected = 0
            self.last_corners = []
            self.last_ids = []
            return False

        if getattr(frame, "size", 0) == 0:
            self.tags_detected = 0
            self.last_corners = []
            self.last_ids = []
            return False

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        corners, ids, _ = self.detector.detectMarkers(gray)

        if corners is None or len(corners) == 0 or ids is None or len(ids) == 0:
            self.tags_detected = 0
            self.last_corners = []
            self.last_ids = []
            return False

        ids_flat = ids.flatten()
        self.tags_detected = len(ids_flat)
        self.last_corners = list(corners)
        self.last_ids = [int(i) for i in ids_flat]

        layout_mm = {} if self.tag_layout_mm is None else self.tag_layout_mm

        img_pts = []
        obj_pts = []
        half_s = self.tag_size_mm / 2.0

        for i, tid in enumerate(ids_flat):
            try:
                tid_int = int(tid)
            except (ValueError, TypeError):
                continue

            pos = layout_mm.get(tid_int)
            if pos is None:
                continue

            try:
                cx, cy = float(pos[0]), float(pos[1])
            except (TypeError, ValueError, IndexError):
                continue

            # Convenção de vértices horários ArUco (0: TL, 1: TR, 2: BR, 3: BL)
            tag_obj = [
                [cx - half_s, cy + half_s],
                [cx + half_s, cy + half_s],
                [cx + half_s, cy - half_s],
                [cx - half_s, cy - half_s]
            ]
            c = corners[i][0]
            for p_idx in range(4):
                img_pts.append(c[p_idx])
                obj_pts.append(tag_obj[p_idx])

        if len(img_pts) < 4:
            return False

        src_pts = np.array(img_pts, dtype=np.float32)
        dst_pts = np.array(obj_pts, dtype=np.float32)

        H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 3.0)
        if H is None or H.shape != (3, 3) or not np.all(np.isfinite(H)):
            return False

        det = np.linalg.det(H)
        if abs(det) < 1e-9:
            return False

        try:
            H_inv = np.linalg.inv(H)
        except np.linalg.LinAlgError:
            return False

        self.H = H
        self.H_inv = H_inv
        self.is_calibrated = True

        # Estimação da inclinação da câmera via solvePnP
        try:
            h_img, w_img = gray.shape[:2]
            cx_img, cy_img = w_img / 2.0, h_img / 2.0
            f_est = float(w_img)
            p0 = layout_mm.get(0)
            p1 = layout_mm.get(1)
            if p0 is not None and p1 is not None and 0 in ids_flat and 1 in ids_flat:
                try:
                    c0_mm = np.array([float(p0[0]), float(p0[1])], dtype=np.float32)
                    c1_mm = np.array([float(p1[0]), float(p1[1])], dtype=np.float32)
                    i0 = list(ids_flat).index(0)
                    i1 = list(ids_flat).index(1)
                    c0 = np.mean(corners[i0][0], axis=0)
                    c1 = np.mean(corners[i1][0], axis=0)
                    px_dist = float(np.linalg.norm(c1 - c0))
                    mm_dist = float(np.linalg.norm(c1_mm - c0_mm))
                    if mm_dist > 1.0 and px_dist > 1.0:
                        f_est = self.cam_height_mm * (px_dist / mm_dist)
                except Exception:
                    pass

            K = np.array([[f_est, 0.0, cx_img], [0.0, f_est, cy_img], [0.0, 0.0, 1.0]], dtype=np.float32)
            obj_3d = np.hstack([dst_pts, np.zeros((len(dst_pts), 1), dtype=np.float32)])
            success, rvec, _ = cv2.solvePnP(obj_3d, src_pts, K, None, flags=cv2.SOLVEPNP_ITERATIVE)
            if success:
                R, _ = cv2.Rodrigues(rvec)
                cos_tilt = abs(float(R[2, 2]))
                self.estimated_cam_tilt_deg = float(np.degrees(np.arccos(np.clip(cos_tilt, -1.0, 1.0))))
            else:
                self.estimated_cam_tilt_deg = 0.0
        except Exception:
            self.estimated_cam_tilt_deg = 0.0

        return True

    def pixel_to_mm(self, px_x: float, px_y: float,
                    cam_height_mm: Optional[float] = None,
                    z_ball: Optional[float] = None) -> Tuple[float, float]:
        """
        Converte coordenadas de pixel para mm da placa aplicando homografia e correção de paralaxe.
        """
        if not self.is_calibrated or self.H is None:
            return 0.0, 0.0

        denom = self.H[2, 0] * px_x + self.H[2, 1] * px_y + self.H[2, 2]
        if abs(denom) < 1e-9:
            return 0.0, 0.0

        raw_x = (self.H[0, 0] * px_x + self.H[0, 1] * px_y + self.H[0, 2]) / denom
        raw_y = (self.H[1, 0] * px_x + self.H[1, 1] * px_y + self.H[1, 2]) / denom

        h = float(cam_height_mm) if cam_height_mm is not None else self.cam_height_mm
        z = float(z_ball) if z_ball is not None else self.ball_radius_mm
        return apply_parallax_correction(raw_x, raw_y, h, z)

    def mm_to_pixel(self, rx_mm: float, ry_mm: float) -> Tuple[float, float]:
        """
        Converte coordenadas físicas da placa (mm) para pixels (para setpoints na superfície da placa z=0).
        """
        if not self.is_calibrated or self.H_inv is None:
            return 0.0, 0.0

        denom = self.H_inv[2, 0] * rx_mm + self.H_inv[2, 1] * ry_mm + self.H_inv[2, 2]
        if abs(denom) < 1e-9:
            return 0.0, 0.0

        px_x = (self.H_inv[0, 0] * rx_mm + self.H_inv[0, 1] * ry_mm + self.H_inv[0, 2]) / denom
        px_y = (self.H_inv[1, 0] * rx_mm + self.H_inv[1, 1] * ry_mm + self.H_inv[1, 2]) / denom
        return float(px_x), float(px_y)


# ==============================================================================
# 4. DETECTOR VISUAL HSV OTIMIZADO
# ==============================================================================
class HSVBallDetector:
    """Detector baseado em limiarização HSV com buffers pré-alocados, wrap-around e circularidade."""

    def __init__(self, hsv_lower: Tuple[int, int, int] = (5, 120, 100),
                 hsv_upper: Tuple[int, int, int] = (25, 255, 255),
                 min_radius: int = 4, max_radius: int = 40,
                 blur_size: int = 5, min_circularity: float = 0.65):
        self.min_radius = int(min_radius)
        self.max_radius = int(max_radius)
        self.blur_size = int(blur_size) if blur_size % 2 == 1 else int(blur_size + 1)
        self.min_circularity = float(min_circularity)

        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)) if cv2 is not None else None
        self.latest_mask = None

        # Buffers NumPy pré-alocados para evitar alocações dinâmicas contínuas
        self._buf_shape = None
        self._buf_blurred = None
        self._buf_hsv = None
        self._buf_mask = None
        self._buf_mask1 = None
        self._buf_mask2 = None

        self._set_bounds(hsv_lower, hsv_upper)

    def get_name(self) -> str:
        return "HSV"

    def _set_bounds(self, lower, upper):
        self._lower_tuple = tuple(int(x) for x in lower)
        self._upper_tuple = tuple(int(x) for x in upper)
        lh, ls, lv = self._lower_tuple
        uh, us, uv = self._upper_tuple

        s_min, s_max = min(ls, us), max(ls, us)
        v_min, v_max = min(lv, uv), max(lv, uv)

        if lh > uh:
            self._is_wrap = True
            self._lower1 = np.array([lh, s_min, v_min], dtype=np.uint8) if np is not None else [lh, s_min, v_min]
            self._upper1 = np.array([179, s_max, v_max], dtype=np.uint8) if np is not None else [179, s_max, v_max]
            self._lower2 = np.array([0, s_min, v_min], dtype=np.uint8) if np is not None else [0, s_min, v_min]
            self._upper2 = np.array([uh, s_max, v_max], dtype=np.uint8) if np is not None else [uh, s_max, v_max]
        else:
            self._is_wrap = False
            self._lower = np.array([lh, s_min, v_min], dtype=np.uint8) if np is not None else [lh, s_min, v_min]
            self._upper = np.array([uh, s_max, v_max], dtype=np.uint8) if np is not None else [uh, s_max, v_max]

    def set_parameters(self, lower=None, upper=None, blur=None,
                       min_radius=None, max_radius=None, min_circularity=None):
        new_lower = lower if lower is not None else self._lower_tuple
        new_upper = upper if upper is not None else self._upper_tuple
        if lower is not None or upper is not None:
            self._set_bounds(new_lower, new_upper)
        if blur is not None:
            b = int(blur)
            self.blur_size = b if b % 2 == 1 else b + 1
        if min_radius is not None:
            self.min_radius = max(1, int(min_radius))
        if max_radius is not None:
            self.max_radius = max(self.min_radius + 1, int(max_radius))
        if min_circularity is not None:
            self.min_circularity = float(min_circularity)

    def _ensure_buffers(self, h: int, w: int):
        if self._buf_shape != (h, w) and np is not None:
            self._buf_shape = (h, w)
            self._buf_blurred = np.empty((h, w, 3), dtype=np.uint8)
            self._buf_hsv = np.empty((h, w, 3), dtype=np.uint8)
            self._buf_mask = np.empty((h, w), dtype=np.uint8)
            self._buf_mask1 = np.empty((h, w), dtype=np.uint8)
            self._buf_mask2 = np.empty((h, w), dtype=np.uint8)

    def detect(self, frame: np.ndarray) -> Tuple[bool, Tuple[float, float, float], float, Optional[Tuple[int, int, int, int]]]:
        if frame is None or cv2 is None or np is None or getattr(frame, "size", 0) == 0:
            return False, (0.0, 0.0, 0.0), 0.0, None

        h, w = frame.shape[:2]
        if h < 2 or w < 2:
            return False, (0.0, 0.0, 0.0), 0.0, None

        self._ensure_buffers(h, w)

        cv2.GaussianBlur(frame, (self.blur_size, self.blur_size), 0, dst=self._buf_blurred)
        cv2.cvtColor(self._buf_blurred, cv2.COLOR_BGR2HSV, dst=self._buf_hsv)

        if self._is_wrap:
            cv2.inRange(self._buf_hsv, self._lower1, self._upper1, dst=self._buf_mask1)
            cv2.inRange(self._buf_hsv, self._lower2, self._upper2, dst=self._buf_mask2)
            cv2.bitwise_or(self._buf_mask1, self._buf_mask2, dst=self._buf_mask)
        else:
            cv2.inRange(self._buf_hsv, self._lower, self._upper, dst=self._buf_mask)

        if self.kernel is not None:
            cv2.morphologyEx(self._buf_mask, cv2.MORPH_OPEN, self.kernel, dst=self._buf_mask)
            cv2.morphologyEx(self._buf_mask, cv2.MORPH_CLOSE, self.kernel, dst=self._buf_mask)

        self.latest_mask = self._buf_mask

        contours, _ = cv2.findContours(self._buf_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, (0.0, 0.0, 0.0), 0.0, None

        best_contour = None
        best_area = 0.0
        best_radius = 0.0
        best_center = (0.0, 0.0)
        best_circularity = 0.0

        for c in contours:
            area = cv2.contourArea(c)
            if area < (math.pi * (self.min_radius ** 2) * 0.4):
                continue
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if not (self.min_radius <= radius <= self.max_radius):
                continue
            perimeter = cv2.arcLength(c, True)
            if perimeter <= 0.0:
                continue
            circularity = (4.0 * math.pi * area) / (perimeter ** 2)
            if circularity < self.min_circularity:
                continue
            if area > best_area:
                best_area = area
                best_contour = c
                best_radius = radius
                best_center = (x, y)
                best_circularity = circularity

        if best_contour is None:
            return False, (0.0, 0.0, 0.0), 0.0, None

        M = cv2.moments(best_contour)
        if M["m00"] > 1e-5:
            cx = float(M["m10"] / M["m00"])
            cy = float(M["m01"] / M["m00"])
        else:
            cx, cy = float(best_center[0]), float(best_center[1])

        x, y, bw, bh = cv2.boundingRect(best_contour)
        confidence = float(min(1.0, max(0.0, best_circularity)))
        return True, (cx, cy, float(best_radius)), confidence, (x, y, bw, bh)


# ==============================================================================
# 5. FÁBRICA DE DETECTORES
# ==============================================================================
def create_detector(detector_type: str = "hsv", **kwargs) -> HSVBallDetector:
    """Fábrica de detectores visuais da placa (retorna sempre HSVBallDetector otimizado)."""
    hsv_lower = kwargs.get("hsv_lower")
    if hsv_lower is None and config is not None:
        hsv_lower = getattr(config, "HSV_LOWER", (5, 120, 100))
    if hsv_lower is None:
        hsv_lower = (5, 120, 100)

    hsv_upper = kwargs.get("hsv_upper")
    if hsv_upper is None and config is not None:
        hsv_upper = getattr(config, "HSV_UPPER", (25, 255, 255))
    if hsv_upper is None:
        hsv_upper = (25, 255, 255)

    min_radius = kwargs.get("min_radius")
    if min_radius is None and config is not None:
        min_radius = getattr(config, "BALL_MIN_RADIUS_PX", 4)
    if min_radius is None:
        min_radius = 4

    max_radius = kwargs.get("max_radius")
    if max_radius is None and config is not None:
        max_radius = getattr(config, "BALL_MAX_RADIUS_PX", 40)
    if max_radius is None:
        max_radius = 40

    blur_size = kwargs.get("blur_size")
    if blur_size is None and config is not None:
        blur_size = getattr(config, "BALL_BLUR_SIZE", 5)
    if blur_size is None:
        blur_size = 5

    min_circularity = kwargs.get("min_circularity")
    if min_circularity is None and config is not None:
        min_circularity = getattr(config, "BALL_MIN_CIRCULARITY", 0.65)
    if min_circularity is None:
        min_circularity = 0.65

    return HSVBallDetector(
        hsv_lower=hsv_lower,
        hsv_upper=hsv_upper,
        min_radius=min_radius,
        max_radius=max_radius,
        blur_size=blur_size,
        min_circularity=min_circularity,
    )
