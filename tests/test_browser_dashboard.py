#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Validação no Navegador Headless Chrome e Ausência de Erros JS (tests/test_browser_dashboard.py)
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import re
import shutil
import subprocess
import pytest


def test_headless_chrome_dom_rendering_and_zero_js_errors(live_server):
    chrome_path = shutil.which("google-chrome") or shutil.which("chromium-browser") or shutil.which("chromium")
    if not chrome_path:
        pytest.skip("Google Chrome / Chromium não encontrado no sistema.")

    url = f"{live_server['url']}/"

    # Executa Google Chrome headless moderno renderizando a página completa e capturando logs
    cmd = [
        chrome_path,
        "--headless=new",
        "--no-sandbox",
        "--disable-gpu",
        "--disable-dev-shm-usage",
        "--blink-settings=imagesEnabled=false",
        "--enable-logging=stderr",
        "--dump-dom",
        url
    ]

    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=15)
    rendered_dom = result.stdout
    stderr_log = result.stderr

    # 1. Validação de Carregamento Completo da Página HTML
    assert "<title>Ball and Plate 3-DoF</title>" in rendered_dom or "Ball and Plate" in rendered_dom
    assert "id=\"scopeCanvas\"" in rendered_dom

    # 2. Validação da Tabela de Telemetria de Motores (R2/R3)
    assert "id=\"actuator-table-body\"" in rendered_dom
    assert "id=\"act-row-1\"" in rendered_dom
    assert "id=\"act-row-2\"" in rendered_dom
    assert "id=\"act-row-3\"" in rendered_dom
    assert "id=\"badge-calib-status\"" in rendered_dom
    assert "id=\"lbl-nominal-neutral\"" in rendered_dom
    assert "id=\"lbl-calib-timestamp\"" in rendered_dom

    # 3. Validação dos Controles de Jog Fino/Médio/Grosso e Inputs Customizados (R3)
    assert "id=\"sel-jog-motor\"" in rendered_dom
    assert "id=\"inp-custom-steps\"" in rendered_dom
    assert "id=\"inp-custom-deg\"" in rendered_dom
    assert "jogSelected(-1)" in rendered_dom
    assert "jogSelected(+1)" in rendered_dom
    assert "jogSelected(-5)" in rendered_dom
    assert "jogSelected(+5)" in rendered_dom
    assert "jogSelected(-20)" in rendered_dom
    assert "jogSelected(+20)" in rendered_dom

    # 4. Validação dos Botões de Calibração Permanente (R3)
    assert "id=\"btn-save-neutral\"" in rendered_dom
    assert "id=\"btn-reset-calib\"" in rendered_dom
    assert "id=\"btn-test-repeat\"" in rendered_dom
    assert "id=\"btn-level-platform\"" in rendered_dom

    # 5. Validação dos Painéis e Controles da Suíte de Bancada (Bench Suite) (R3)
    assert "id=\"btn-bench-cardinal\"" in rendered_dom
    assert "id=\"sel-bench-trajectory\"" in rendered_dom
    assert "id=\"btn-bench-trajectory\"" in rendered_dom
    assert "id=\"sel-bench-diag-motor\"" in rendered_dom
    assert "id=\"btn-bench-diagnostics\"" in rendered_dom
    assert "id=\"bench-progress-bar\"" in rendered_dom
    assert "id=\"bench-status-msg\"" in rendered_dom
    assert "id=\"btn-bench-stop\"" in rendered_dom

    # 6. Validação dos Botões de Persistência de Perfil Visual e Controle (R2/R3)
    assert "id=\"btn-save-vision-profile\"" in rendered_dom
    assert "id=\"btn-load-vision-profile\"" in rendered_dom
    assert "id=\"btn-save-control-profile\"" in rendered_dom
    assert "id=\"btn-load-control-profile\"" in rendered_dom

    # 7. Validação de Ausência de Erros no Console JavaScript (ZERO JS Errors)
    # Procura por linhas de log CONSOLE contendo "Uncaught" ou "Error"
    js_errors = []
    for line in stderr_log.splitlines():
        if ":CONSOLE" in line:
            # Verifica se é uma exceção não tratada ou erro de script
            if re.search(r"Uncaught\s+\w*Error", line, re.IGNORECASE) or "SyntaxError" in line or "TypeError" in line or "ReferenceError" in line:
                js_errors.append(line.strip())

    assert len(js_errors) == 0, f"Erros detectados no console JavaScript do navegador: {js_errors}"
