#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Suíte Unificada de Testes e Calibração de Bancada (tools/bench_suite.py)
Testes Cardeais, Trajetórias, Diagnóstico Elétrico, Calibração e Exportação YOLO
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import os
import sys
import time
import math
import argparse
from typing import Optional

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

import config
from core.control_math import HexagonalKinematics3DoF
from core.hardware import MotorController, SerialBridge, save_calibration, load_calibration

try:
    import pigpio
except ImportError:
    pigpio = None


def clear_screen():
    os.system("cls" if os.name == "nt" else "clear")


# ==============================================================================
# 1. TESTES DE MOVIMENTO CARDEAL E TRAJETÓRIAS
# ==============================================================================
def run_full_amplitude_test(kin: HexagonalKinematics3DoF, motor_ctrl: MotorController, max_deg=12.0):
    print("\n" + "=" * 70)
    print("🚀 [TESTE 1] TESTE DE AMPLITUDE TOTAL (LIMITES CARDEAIS)")
    print(f"Inclinação máxima: +/- {max_deg:.1f}°")
    print("=" * 70)

    cardinal_points = [
        ("Neutro (0°, 0°)", 0.0, 0.0),
        (f"Pitch Positivo (+{max_deg}°)", math.radians(max_deg), 0.0),
        ("Neutro (0°, 0°)", 0.0, 0.0),
        (f"Pitch Negativo (-{max_deg}°)", math.radians(-max_deg), 0.0),
        ("Neutro (0°, 0°)", 0.0, 0.0),
        (f"Roll Positivo (+{max_deg}°)", 0.0, math.radians(max_deg)),
        ("Neutro (0°, 0°)", 0.0, 0.0),
        (f"Roll Negativo (-{max_deg}°)", 0.0, math.radians(-max_deg)),
        ("Neutro Final (0°, 0°)", 0.0, 0.0)
    ]

    for desc, p_rad, r_rad in cardinal_points:
        print(f"\n>> Ponto: {desc}")
        ok, res = kin.compute_inverse(p_rad, r_rad)
        if ok:
            motor_ctrl.set_targets(res["step_deltas"])
            motor_ctrl.wait_until_idle(timeout=4.0)
        time.sleep(1.0)
    print("\n✅ Teste de amplitude total concluído.")


def run_programmed_trajectories(kin: HexagonalKinematics3DoF, motor_ctrl: MotorController):
    print("\n" + "=" * 70)
    print("🔄 [TESTE 2] TRAJETÓRIAS PROGRAMADAS")
    print("1. Trajetória Circular Contínua")
    print("2. Trajetória Triangular")
    print("3. Trajetória em Oito (Figura-8)")
    print("=" * 70)
    op = input("Selecione a trajetória (1-3): ").strip()

    radius_deg = 8.0
    r_rad = math.radians(radius_deg)

    if op == "1":
        print(f"\nExecutando 2 voltas circulares (R = {radius_deg}°)...")
        steps = 120
        for loop in range(2):
            for i in range(steps):
                angle = (2.0 * math.pi * i) / steps
                p = r_rad * math.cos(angle)
                r = r_rad * math.sin(angle)
                ok, res = kin.compute_inverse(p, r)
                if ok:
                    motor_ctrl.set_targets(res["step_deltas"])
                time.sleep(0.04)

    elif op == "2":
        print("\nExecutando triângulo equilátero...")
        vertices = [
            (r_rad, 0.0),
            (-0.5 * r_rad, r_rad * math.sqrt(3) / 2.0),
            (-0.5 * r_rad, -r_rad * math.sqrt(3) / 2.0),
            (r_rad, 0.0)
        ]
        for p, r in vertices:
            ok, res = kin.compute_inverse(p, r)
            if ok:
                motor_ctrl.set_targets(res["step_deltas"])
                motor_ctrl.wait_until_idle(timeout=3.0)
            time.sleep(0.5)

    elif op == "3":
        print(f"\nExecutando Figura-8 (Lissajous)...")
        steps = 150
        for i in range(steps):
            t = (2.0 * math.pi * i) / steps
            p = r_rad * math.sin(t)
            r = r_rad * math.sin(2.0 * t) / 2.0
            ok, res = kin.compute_inverse(p, r)
            if ok:
                motor_ctrl.set_targets(res["step_deltas"])
            time.sleep(0.04)

    # Retorna ao repouso
    ok, res = kin.compute_inverse(0.0, 0.0)
    if ok:
        motor_ctrl.set_targets(res["step_deltas"])
        motor_ctrl.wait_until_idle(timeout=3.0)
    print("\n✅ Trajetória concluída.")


def run_line_diagnostics(motor_ctrl: MotorController):
    print("\n" + "=" * 70)
    print("⚡ [TESTE 6] DIAGNÓSTICO ELÉTRICO DE LINHAS GPIO (EN, DIR, STEP)")
    print("=" * 70)
    if not motor_ctrl.is_connected():
        print("❌ pigpio não conectado. Este teste requer conexão ao daemon pigpiod no RPi.")
        return

    pi = motor_ctrl.pi
    for m in (1, 2, 3):
        cfg = config.PIGPIO_MOTORS[m]
        print(f"\nTestando Motor {m} (EN={cfg['en']}, DIR={cfg['dir']}, STEP={cfg['step']})...")
        # 1. Habilita
        pi.write(cfg['en'], 0)
        time.sleep(0.2)
        # 2. Direção avanço
        pi.write(cfg['dir'], 0)
        time.sleep(0.05)
        # 3. 20 pulsos de passo
        for _ in range(20):
            pi.write(cfg['step'], 1)
            time.sleep(0.001)
            pi.write(cfg['step'], 0)
            time.sleep(0.001)
        time.sleep(0.2)
        # 4. Direção retorno
        pi.write(cfg['dir'], 1)
        time.sleep(0.05)
        for _ in range(20):
            pi.write(cfg['step'], 1)
            time.sleep(0.001)
            pi.write(cfg['step'], 0)
            time.sleep(0.001)
        time.sleep(0.2)
        # Desabilita
        pi.write(cfg['en'], 1)
        print(f"  ✓ Motor {m} pulsado com sucesso nos dois sentidos.")
    print("\n✅ Diagnóstico elétrico concluído.")


# ==============================================================================
# 2. CALIBRAÇÃO INTERATIVA DE ÂNGULOS INICIAIS
# ==============================================================================
def run_interactive_calibration(motor_ctrl: MotorController):
    clear_screen()
    calib_data = motor_ctrl.load_calibration_offsets()
    selected_motor = 1

    print("=" * 76)
    print("🎯 ROTINA INTERATIVA DE CALIBRAÇÃO E NIVELAMENTO (Ball and Plate 3-DoF)")
    print("=" * 76)

    try:
        while True:
            offsets = motor_ctrl.calib_offsets
            angles = motor_ctrl.get_motor_angles_deg()
            print(f"\n--- Motor Selecionado: {'TODOS' if selected_motor == 0 else f'Motor {selected_motor}'} ---")
            print(f"  M1 (0°):   Offset={offsets[0]:+d} passos | Ângulo={angles[0]:.2f}°")
            print(f"  M2 (120°): Offset={offsets[1]:+d} passos | Ângulo={angles[1]:.2f}°")
            print(f"  M3 (240°): Offset={offsets[2]:+d} passos | Ângulo={angles[2]:.2f}°")
            print("-" * 76)
            print("  Comandos:")
            print("   [1/2/3/0] Selecionar Motor (0 = Todos)")
            print("   [Q/W] -1/+1 passo (~0.22°)   |   [A/S] -5/+5 passos (~1.13°)")
            print("   [Z/X] -20/+20 passos (~4.5°) |   [T] Alternar Torque (Travar/Soltar)")
            print("   [K] SALVAR CALIBRAÇÃO ATUAL EM JSON")
            print("   [E / SAIR] Encerrar Calibração")
            print("=" * 76)

            cmd = input("Digite o comando: ").strip().lower()
            if not cmd:
                continue

            if cmd in ["1", "2", "3"]:
                selected_motor = int(cmd)
            elif cmd in ["0", "all", "todos"]:
                selected_motor = 0
            elif cmd in ["q", "w", "a", "s", "z", "x"]:
                step_map = {"q": -1, "w": 1, "a": -5, "s": 5, "z": -20, "x": 20}
                delta = step_map[cmd]
                if selected_motor == 0:
                    for m in (1, 2, 3): motor_ctrl.jog(m, delta)
                else:
                    motor_ctrl.jog(selected_motor, delta)
            elif cmd == "t":
                if motor_ctrl.is_enabled:
                    motor_ctrl.disable_all()
                    print("Torque desativado.")
                else:
                    motor_ctrl.enable_all()
                    print("Torque ativado.")
            elif cmd == "k":
                motor_ctrl.save_current_as_neutral()
                print("✅ Calibração persistida no JSON.")
                time.sleep(1.0)
            elif cmd in ["e", "esc", "q!", "sair"]:
                print("Encerrando rotina de calibração...")
                break
    finally:
        motor_ctrl.apply_saved_offsets(wait=True)


# ==============================================================================
# 3. EXPORTAÇÃO YOLO PARA ONNX (DESENVOLVIMENTO)
# ==============================================================================
def export_yolo_onnx(model_name: str = "yolov8n.pt", input_size: int = 224, output_dir: str = "models"):
    try:
        from ultralytics import YOLO
    except ImportError:
        print("❌ Biblioteca 'ultralytics' não encontrada. Instale com: pip install ultralytics onnx")
        return

    os.makedirs(output_dir, exist_ok=True)
    target_onnx = os.path.join(output_dir, "ball_detector_yolo.onnx")
    print(f"[*] Carregando {model_name} e exportando para ONNX ({input_size}x{input_size})...")
    model = YOLO(model_name)
    exported = model.export(format="onnx", imgsz=input_size, simplify=True, dynamic=False, opset=12)
    if os.path.exists(exported):
        import shutil
        shutil.move(exported, target_onnx)
        print(f"✅ Modelo gerado: {target_onnx} ({os.path.getsize(target_onnx)/(1024*1024):.2f} MB)")


# ==============================================================================
# MENU PRINCIPAL DE BANCADA
# ==============================================================================
def main():
    parser = argparse.ArgumentParser(description="Suíte Unificada de Testes e Calibração Ball and Plate 3-DoF")
    parser.add_argument("--microsteps", type=int, default=config.DEFAULT_MICROSTEPS, help="Micropassos TMC2209")
    parser.add_argument("--front-motor", type=int, default=config.MOTOR_FRONT_ID, help="Motor frontal")
    parser.add_argument("--export-yolo", action="store_true", help="Executa exportação ONNX de modelo YOLO")
    parser.add_argument("--model", type=str, default="yolov8n.pt", help="Modelo para exportação YOLO")
    parser.add_argument("--size", type=int, default=224, help="Resolução de entrada YOLO")
    args = parser.parse_args()

    if args.export_yolo:
        export_yolo_onnx(args.model, args.size)
        return

    steps_per_rev = 200 * args.microsteps
    kin = HexagonalKinematics3DoF(steps_per_rev=steps_per_rev, front_motor_id=args.front_motor)
    motor_ctrl = MotorController(steps_per_rev=steps_per_rev)
    motor_ctrl.connect()

    while True:
        print("\n" + "=" * 70)
        print("🛠️  SUÍTE UNIFICADA DE TESTES E CALIBRAÇÃO (BALL AND PLATE 3-DoF)")
        print(f"Modo: STANDALONE_RPI | pigpio: {'CONECTADO' if motor_ctrl.is_connected() else 'DESCONECTADO'}")
        print("=" * 70)
        print("  [1] Teste de Amplitude Total (Limites Físicos Cardeais +/- 12°)")
        print("  [2] Trajetórias Programadas (Círculo, Triângulo, Figura-8)")
        print("  [3] Teste com Entrada Manual de Ângulo (Pitch e Roll em Graus)")
        print("  [4] Diagnóstico Elétrico de Linhas (EN, DIR, STEP dos 3 Motores)")
        print("  [5] Rotina Interativa de Calibração de Neutro e Gravação JSON")
        print("  [6] Alternar Torque das Bobinas (Travar / Soltar Eixos)")
        print("  [7] Nivelar Plataforma (0°, 0°)")
        print("  [0] Sair")
        print("=" * 70)

        op = input("Selecione uma opção: ").strip()
        if op == "1":
            run_full_amplitude_test(kin, motor_ctrl)
        elif op == "2":
            run_programmed_trajectories(kin, motor_ctrl)
        elif op == "3":
            try:
                p = float(input("Pitch desejado (graus, ex: 3.5): "))
                r = float(input("Roll desejado (graus, ex: -2.0): "))
                ok, res = kin.compute_inverse(math.radians(p), math.radians(r))
                if ok:
                    motor_ctrl.set_targets(res["step_deltas"])
                    motor_ctrl.wait_until_idle()
                    print(f"Comando aplicado: M1={res['step_deltas'][0]}, M2={res['step_deltas'][1]}, M3={res['step_deltas'][2]}")
                else:
                    print("Ângulo fora do envelope cinemático!")
            except ValueError:
                print("Valor inválido.")
        elif op == "4":
            run_line_diagnostics(motor_ctrl)
        elif op == "5":
            run_interactive_calibration(motor_ctrl)
        elif op == "6":
            if motor_ctrl.is_enabled:
                motor_ctrl.disable_all()
                print("Bobinas desenergizadas.")
            else:
                motor_ctrl.enable_all()
                print("Bobinas travadas com torque.")
        elif op == "7":
            ok, res = kin.compute_inverse(0.0, 0.0)
            if ok:
                motor_ctrl.set_targets(res["step_deltas"])
                motor_ctrl.wait_until_idle()
            print("Plataforma nivelada.")
        elif op == "0":
            break

    motor_ctrl.disable_all()
    motor_ctrl.stop()
    print("Suíte encerrada com segurança.")


if __name__ == "__main__":
    main()
