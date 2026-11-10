#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Suíte Unificada de Testes e Calibração de Bancada (tools/bench_suite.py)
Testes Cardeais, Trajetórias, Diagnóstico Elétrico e Calibração
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import os
import sys
import time
import math
import argparse

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

import config
from core.control_math import HexagonalKinematics3DoF
from core.hardware import (
    MotorController,
    get_default_calibration_path,
)


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
        (f"Diagonal Nordeste (+{max_deg*0.7:.1f}°, +{max_deg*0.7:.1f}°)",
         math.radians(max_deg * 0.7), math.radians(max_deg * 0.7)),
        (f"Diagonal Sudoeste (-{max_deg*0.7:.1f}°, -{max_deg*0.7:.1f}°)",
         math.radians(-max_deg * 0.7), math.radians(-max_deg * 0.7)),
        ("Neutro Final (0°, 0°)", 0.0, 0.0)
    ]

    for desc, p_rad, r_rad in cardinal_points:
        ok, res = kin.compute_inverse(p_rad, r_rad)
        if not ok:
            print(f"  ❌ {desc} -> Posição fora dos limites cinemáticos!")
            continue
        stp = res["step_deltas"]
        ang_deg = [round(a, 1) for a in res["angles_deg"]]
        print(f"  -> Movendo para: {desc:<36s} | Passos: {stp} | Braços: {ang_deg}°")
        motor_ctrl.set_targets(stp)
        motor_ctrl.wait_until_idle(timeout=3.0)
        time.sleep(0.3)

    print("\n✅ Teste de amplitude total concluído.")


def run_programmed_trajectories(kin: HexagonalKinematics3DoF, motor_ctrl: MotorController):
    print("\n" + "=" * 70)
    print("🔄 [TESTE 2] TRAJETÓRIAS PROGRAMADAS")
    print("  [1] Trajetória Circular Contínua")
    print("  [2] Trajetória Triangular")
    print("  [3] Trajetória Quadrada")
    print("  [4] Trajetória em Oito (Figura-8 / Lissajous)")
    print("=" * 70)
    op = input("Selecione a trajetória [1-4, padrão: 1]: ").strip() or "1"

    radius_deg = 6.0
    r_rad = math.radians(radius_deg)

    if op == "1":
        print(f"\nExecutando 2 voltas circulares (R = {radius_deg}°)...")
        steps = 60
        for _ in range(2):
            for i in range(steps):
                angle = (2.0 * math.pi * i) / steps
                p = r_rad * math.cos(angle)
                r = r_rad * math.sin(angle)
                ok, res = kin.compute_inverse(p, r)
                if ok:
                    motor_ctrl.set_targets(res["step_deltas"])
                time.sleep(0.06)

    elif op == "2":
        print("\nExecutando 2 ciclos triangulares...")
        vertices = [
            (r_rad, 0.0),
            (-0.5 * r_rad, r_rad * math.sqrt(3) / 2.0),
            (-0.5 * r_rad, -r_rad * math.sqrt(3) / 2.0),
            (r_rad, 0.0)
        ]
        for _ in range(2):
            for v_idx in range(3):
                p0, r0 = vertices[v_idx]
                p1, r1 = vertices[v_idx + 1]
                for s in range(15):
                    t = s / 15.0
                    ok, res = kin.compute_inverse(p0 + t * (p1 - p0), r0 + t * (r1 - r0))
                    if ok:
                        motor_ctrl.set_targets(res["step_deltas"])
                    time.sleep(0.06)

    elif op == "3":
        print("\nExecutando 2 ciclos quadrados...")
        vertices = [
            (r_rad, r_rad), (-r_rad, r_rad),
            (-r_rad, -r_rad), (r_rad, -r_rad), (r_rad, r_rad)
        ]
        for _ in range(2):
            for v_idx in range(4):
                p0, r0 = vertices[v_idx]
                p1, r1 = vertices[v_idx + 1]
                for s in range(12):
                    t = s / 12.0
                    ok, res = kin.compute_inverse(p0 + t * (p1 - p0), r0 + t * (r1 - r0))
                    if ok:
                        motor_ctrl.set_targets(res["step_deltas"])
                    time.sleep(0.06)

    elif op == "4":
        print("\nExecutando 2 ciclos em Figura-8 (Lissajous)...")
        steps = 80
        for _ in range(2):
            for i in range(steps):
                t = (2.0 * math.pi * i) / steps
                p = r_rad * math.sin(t)
                r = r_rad * math.sin(2.0 * t) * 0.5
                ok, res = kin.compute_inverse(p, r)
                if ok:
                    motor_ctrl.set_targets(res["step_deltas"])
                time.sleep(0.06)
    else:
        print(f"\n❌ Opção de trajetória inválida: '{op}'. Escolha entre 1 e 4.")
        return

    motor_ctrl.level()
    motor_ctrl.wait_until_idle(timeout=3.0)
    print("\n✅ Trajetória concluída e plataforma retornada ao neutro.")


def run_line_diagnostics(motor_ctrl: MotorController):
    print("\n" + "=" * 70)
    print("🔬 [TESTE 4] DIAGNÓSTICO ELÉTRICO DE LINHAS GPIO (EN, DIR, STEP)")
    print("=" * 70)
    if not motor_ctrl.is_connected():
        print("❌ pigpio não conectado. Inicie o daemon: sudo pigpiod")
        return

    print("Selecione o motor: [1] Motor 1 | [2] Motor 2 | [3] Motor 3 | [0] Todos")
    m_choice = input("Opção [0-3, padrão: 0]: ").strip()
    motors = [int(m_choice)] if m_choice in ("1", "2", "3") else [1, 2, 3]

    for m in motors:
        cfg = config.PIGPIO_MOTORS.get(m, {})
        name = cfg.get("name", f"Motor {m}")
        print(f"\n>>> Testando {name} (DIR=GPIO{cfg.get('dir')}, STEP=GPIO{cfg.get('step')}, EN=GPIO{cfg.get('en')})")
        print("  [1/2] Avançando +80 micropassos...")
        motor_ctrl.jog(m, 80)
        motor_ctrl.wait_until_idle(timeout=2.5)
        time.sleep(0.5)
        print("  [2/2] Retornando -80 micropassos ao ponto inicial...")
        motor_ctrl.jog(m, -80)
        motor_ctrl.wait_until_idle(timeout=2.5)
        print(f"  ✓ {name} testado nos dois sentidos.")

    print("\n✅ Diagnóstico elétrico concluído.")


# ==============================================================================
# 2. CALIBRAÇÃO INTERATIVA DE ÂNGULOS INICIAIS (INTERFACE AMIGÁVEL COMPLETA)
# ==============================================================================
def _print_calibration_header(calib_data: dict, steps_per_rev: int):
    print("=" * 76)
    print("  📐 ROTINA INTERATIVA DE MEDIÇÃO E CALIBRAÇÃO DOS ÂNGULOS INICIAIS")
    print(f"  Plataforma Ball and Plate 3-DoF | 3x NEMA 17 + TMC2209 ({steps_per_rev} passos/rev)")
    print("=" * 76)
    status_str = "CALIBRADO" if calib_data.get("is_calibrated") else "PADRÃO (SEM CALIBRAÇÃO)"
    ts = calib_data.get("timestamp") or "Nenhum registo"
    print(f"  Status do Arquivo: {status_str} | Último Salvo: {ts}")
    print(f"  Arquivo JSON: {get_default_calibration_path()}")
    print("=" * 76)


def _print_telemetry_table(motor_ctrl: MotorController, selected_motor: int):
    deg_per_step = 360.0 / float(motor_ctrl.steps_per_rev)
    nom_deg = motor_ctrl.nominal_neutral_deg
    calib_offsets = motor_ctrl.calib_offsets

    sep = "  +------+------------------------------+-------------+------------+--------------+---------------------+"
    hdr = "  | ID   | Atuador                      | Pinos GPIO  | Passo Rel. | Offset Salvo | Ângulo Total (Delta)|"

    print("\n  TABELA DE ESTADO E ÂNGULOS DOS ATUADORES:")
    print(sep)
    print(hdr)
    print(sep)

    for mid in (1, 2, 3):
        cfg = config.PIGPIO_MOTORS.get(mid, {})
        idx = mid - 1
        cur_s = motor_ctrl.current_steps[idx]
        saved_s = calib_offsets[idx]
        total_s = saved_s + cur_s
        total_deg = nom_deg + (total_s * deg_per_step)
        delta_deg = total_s * deg_per_step

        marker = "->" if (selected_motor == mid or selected_motor == 0) else "  "
        pin_info = f"D:{cfg.get('dir', '-')} S:{cfg.get('step', '-')}"
        name = cfg.get("name", f"Motor {mid}")
        print(f"  | {marker} {mid} | {name:<28} | {pin_info:<11} | {cur_s:+5d} stp  | {saved_s:+5d} stp    | {total_deg:6.2f}° ({delta_deg:+5.2f}°)  |")

    print(sep)
    sel_label = "TODOS OS MOTORES SIMULTANEAMENTE" if selected_motor == 0 else config.PIGPIO_MOTORS.get(selected_motor, {}).get("name", f"Motor {selected_motor}")
    torque_str = "TRAVADO COM TORQUE (EN=0)" if motor_ctrl.is_enabled else "LIVRE / SOLTO (EN=1)"
    print(f"  Alvo de Ajuste Selecionado: [{sel_label}]")
    print(f"  Estado das Bobinas: {torque_str}")
    print("-" * 76)


def _print_controls_menu():
    print("  COMANDOS DE AJUSTE FINO E CONTROLE:")
    print("   [1] Selecionar Motor 1 (0°)        [2] Selecionar Motor 2 (120°)")
    print("   [3] Selecionar Motor 3 (240°)      [0] Selecionar TODOS os Motores")
    print("   -------------------------------------------------------------------")
    print("   Ajuste Fino:    [Q] -1 passo   (~0.22°)  |  [W] +1 passo   (~0.22°)")
    print("   Ajuste Médio:   [A] -5 passos  (~1.13°)  |  [S] +5 passos  (~1.13°)")
    print("   Ajuste Grosso:  [Z] -20 passos (~4.50°)  |  [X] +20 passos (~4.50°)")
    print("   [J] Digitar quantidade manual de passos (ex: +12, -45)")
    print("   [G] Digitar ângulo desejado em graus (ex: +1.5, -2.0)")
    print("   -------------------------------------------------------------------")
    print("   [T] Alternar Torque (Soltar/Travar para alinhamento mecânico manual)")
    print("   [V] Testar Nivelamento (Afasta +60 passos e retorna ao ponto calibrado)")
    print("   [K] SALVAR CALIBRAÇÃO ATUAL EM ARQUIVO JSON (Gravação Permanente)")
    print("   [R] RESETAR CALIBRAÇÃO PARA ZEROS (Restaurar Padrão de Fábrica)")
    print("   [E / Esc] Sair da Calibração")
    print("=" * 76)


def run_interactive_calibration(motor_ctrl: MotorController = None):
    """Executa a rotina interativa completa e amigável de calibração dos ângulos iniciais."""
    created_locally = False
    if motor_ctrl is None:
        motor_ctrl = MotorController(steps_per_rev=1600)
        motor_ctrl.connect(force=True)
        created_locally = True
    elif not motor_ctrl.is_connected():
        motor_ctrl.connect(force=True)

    if not motor_ctrl.is_connected():
        print("\n⚠️ AVISO: pigpio não conectado (daemon pigpiod inativo).")
        print("   Os ajustes na interface serão virtuais (sem acionamento físico dos motores).")
        print("   Para acionar os motores reais no Raspberry Pi, execute em outro terminal: sudo pigpiod\n")
        time.sleep(1.2)
    else:
        motor_ctrl.enable_all()

    calib_data = motor_ctrl.load_calibration_offsets()
    selected_motor = 1

    try:
        while True:
            clear_screen()
            _print_calibration_header(calib_data, motor_ctrl.steps_per_rev)
            _print_telemetry_table(motor_ctrl, selected_motor)
            _print_controls_menu()

            cmd = input("Digite o comando desejado: ").strip().lower()
            if not cmd:
                continue

            if cmd in ("1", "2", "3"):
                selected_motor = int(cmd)
            elif cmd in ("0", "all", "todos"):
                selected_motor = 0

            elif cmd in ("q", "w", "a", "s", "z", "x"):
                step_map = {"q": -1, "w": +1, "a": -5, "s": +5, "z": -20, "x": +20}
                delta = step_map[cmd]
                targets = (1, 2, 3) if selected_motor == 0 else (selected_motor,)
                for m in targets:
                    motor_ctrl.jog(m, delta)
                motor_ctrl.wait_until_idle(timeout=1.5)

            elif cmd == "j":
                raw = input("Digite a quantidade de passos a avançar (+) ou recuar (-): ").strip()
                try:
                    delta = int(raw)
                    targets = (1, 2, 3) if selected_motor == 0 else (selected_motor,)
                    for m in targets:
                        motor_ctrl.jog(m, delta)
                    motor_ctrl.wait_until_idle(timeout=2.5)
                except ValueError:
                    print("❌ Valor inválido.")
                    time.sleep(1.0)

            elif cmd == "g":
                raw = input("Digite a variação angular desejada em graus (ex: +2.5): ").strip()
                try:
                    deg = float(raw)
                    steps_per_deg = motor_ctrl.steps_per_rev / 360.0
                    delta = int(round(deg * steps_per_deg))
                    targets = (1, 2, 3) if selected_motor == 0 else (selected_motor,)
                    for m in targets:
                        motor_ctrl.jog(m, delta)
                    motor_ctrl.wait_until_idle(timeout=2.5)
                except ValueError:
                    print("❌ Valor numérico inválido.")
                    time.sleep(1.0)

            elif cmd == "t":
                if motor_ctrl.is_enabled:
                    motor_ctrl.disable_all()
                    print("\n🔓 Bobinas desligadas. Pode ajustar os braços manualmente.")
                else:
                    if not motor_ctrl.is_connected() and not motor_ctrl.connect(force=True):
                        print("\n❌ pigpio não conectado. Inicie com: sudo pigpiod")
                    else:
                        motor_ctrl.enable_all()
                        print("\n🔒 Bobinas ligadas e travadas com torque.")
                time.sleep(1.2)

            elif cmd == "v":
                print("\n🧪 Executando teste de repetibilidade de nivelamento...")
                print("   1. Afastando +60 passos temporariamente...")
                for m in (1, 2, 3):
                    motor_ctrl.jog(m, 60)
                motor_ctrl.wait_until_idle(timeout=2.0)
                time.sleep(0.5)

                print("   2. Retornando ao ponto neutro calibrado (level)...")
                motor_ctrl.level()
                motor_ctrl.wait_until_idle(timeout=2.0)
                print("   ✅ Plataforma retornada ao neutro. Verifique com o nível de bolha.")
                input("\nPressione ENTER para continuar...")

            elif cmd == "k":
                confirm = input("\n💾 Deseja gravar a posição atual como o novo neutro permanente? (S/n): ").strip().lower()
                if confirm in ("", "s", "sim", "y"):
                    if motor_ctrl.save_current_as_neutral():
                        calib_data = motor_ctrl.load_calibration_offsets()
                        print("\n🎉 Calibração salva com sucesso no JSON!")
                    else:
                        print("\n❌ Falha ao gravar o ficheiro de calibração.")
                    time.sleep(1.5)

            elif cmd == "r":
                confirm = input("\n⚠️ Tem a certeza que deseja resetar os offsets para zero (Padrão de Fábrica)? (s/N): ").strip().lower()
                if confirm in ("s", "sim", "y"):
                    if motor_ctrl.reset_calibration():
                        calib_data = motor_ctrl.load_calibration_offsets()
                        print("\n🔄 Offsets resetados para [0, 0, 0] com sucesso!")
                    else:
                        print("\n❌ Erro ao resetar o ficheiro de calibração.")
                    time.sleep(1.5)

            elif cmd in ("e", "esc", "q!", "sair"):
                print("\nEncerrando rotina de calibração e retornando ao neutro...")
                motor_ctrl.level()
                motor_ctrl.wait_until_idle(timeout=2.0)
                break
    finally:
        if created_locally:
            motor_ctrl.disable_all()
            motor_ctrl.stop()


# ==============================================================================
# MENU PRINCIPAL DE BANCADA
# ==============================================================================
def main():
    parser = argparse.ArgumentParser(description="Suíte Unificada de Testes e Calibração Ball and Plate 3-DoF")
    parser.add_argument("--microsteps", type=int, default=config.DEFAULT_MICROSTEPS, help="Micropassos TMC2209")
    parser.add_argument("--front-motor", type=int, default=config.MOTOR_FRONT_ID, help="Motor frontal")
    parser.add_argument("--apply-calibration", action="store_true", default=True, help="Aplica offsets salvos do JSON ao iniciar (padrão: True)")
    parser.add_argument("--no-calibration", dest="apply_calibration", action="store_false", help="Ignora offsets salvos do JSON")
    parser.add_argument("--calibrate", action="store_true", help="Abre diretamente a rotina interativa de calibração")
    args = parser.parse_args()

    steps_per_rev = 200 * args.microsteps
    kin = HexagonalKinematics3DoF(steps_per_rev=steps_per_rev, front_motor_id=args.front_motor)
    motor_ctrl = MotorController(steps_per_rev=steps_per_rev, apply_offsets_on_init=args.apply_calibration)
    motor_ctrl.connect(force=True)

    if args.calibrate:
        try:
            run_interactive_calibration(motor_ctrl)
        finally:
            motor_ctrl.disable_all()
            motor_ctrl.stop()
        return

    try:
        while True:
            print("\n" + "=" * 72)
            print("🛠️  SUÍTE UNIFICADA DE TESTES E CALIBRAÇÃO (BALL AND PLATE 3-DoF)")
            print(f"Modo: STANDALONE_RPI | pigpio: {'CONECTADO' if motor_ctrl.is_connected() else 'DESCONECTADO'} | Neutro: {kin.neutral_angle_deg:.2f}°")
            print("=" * 72)
            print("  [1] Teste de Amplitude Total (Limites Físicos Cardeais +/- 12°)")
            print("  [2] Trajetórias Programadas (Círculo, Triângulo, Quadrado, Figura-8)")
            print("  [3] Teste com Entrada Manual de Ângulo (Pitch e Roll em Graus)")
            print("  [4] Diagnóstico Elétrico de Linhas (EN, DIR, STEP dos 3 Motores)")
            print("  [5] Calibração Interativa dos Ângulos Iniciais (Ajuste Fino, Gravação e Reset)")
            print("  [6] Alternar Torque das Bobinas (Travar / Soltar Eixos)")
            print("  [7] Nivelar Plataforma (Retornar todos os motores para Zero)")
            print("  [0] Sair")
            print("=" * 72)

            op = input("Selecione uma opção [0-7]: ").strip()
            if op == "1":
                run_full_amplitude_test(kin, motor_ctrl)
            elif op == "2":
                run_programmed_trajectories(kin, motor_ctrl)
            elif op == "3":
                try:
                    raw_p = input("Pitch desejado em graus [-15 a +15, padrão: 0]: ").strip()
                    p = float(raw_p) if raw_p else 0.0
                    raw_r = input("Roll desejado em graus [-15 a +15, padrão: 0]: ").strip()
                    r = float(raw_r) if raw_r else 0.0
                    ok, res = kin.compute_inverse(math.radians(p), math.radians(r))
                    if ok:
                        motor_ctrl.set_targets(res["step_deltas"])
                        motor_ctrl.wait_until_idle(timeout=3.0)
                        print(f"  -> Braços: {res['angles_deg']}° | Passos: {res['step_deltas']}")
                    else:
                        print("  ❌ Ângulo fora do envelope cinemático!")
                except ValueError:
                    print("  ❌ Valor numérico inválido.")
            elif op == "4":
                run_line_diagnostics(motor_ctrl)
            elif op == "5":
                run_interactive_calibration(motor_ctrl)
            elif op == "6":
                if motor_ctrl.is_enabled:
                    motor_ctrl.disable_all()
                    print("🔓 Bobinas desenergizadas (eixos livres).")
                else:
                    if not motor_ctrl.is_connected() and not motor_ctrl.connect(force=True):
                        print("❌ pigpio não conectado. Inicie com: sudo pigpiod")
                    else:
                        motor_ctrl.enable_all()
                        print("🔒 Bobinas travadas com torque.")
            elif op == "7":
                motor_ctrl.level()
                motor_ctrl.wait_until_idle(timeout=3.0)
                print("✅ Plataforma nivelada no neutro (0, 0, 0).")
            elif op == "0":
                break
    finally:
        motor_ctrl.disable_all()
        motor_ctrl.stop()
        print("Suíte encerrada com segurança.")


if __name__ == "__main__":
    main()