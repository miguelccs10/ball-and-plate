#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Módulo de Persistência Atômica e Segura (core/persistence.py)
Gravação atômica via tempfile + os.fsync + os.replace para proteção contra
corrupção por quedas abruptas de energia ou encerramento forçado.
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import os
import json
import tempfile
from datetime import datetime
from typing import Any, Dict, Optional


def atomic_write_json(filepath: str, data: Dict[str, Any]) -> bool:
    """
    Grava dados em formato JSON de forma estritamente atômica.
    Cria arquivo temporário no mesmo diretório do arquivo de destino (garantindo
    o mesmo filesystem/ponto de montagem para o os.replace), força escrita em
    disco via fsync e realiza a substituição atômica via os.replace.
    """
    dir_name = os.path.dirname(os.path.abspath(filepath))
    os.makedirs(dir_name, exist_ok=True)
    temp_file = None
    try:
        with tempfile.NamedTemporaryFile("w", dir=dir_name, delete=False, encoding="utf-8") as tf:
            temp_file = tf.name
            json.dump(data, tf, indent=2, ensure_ascii=False)
            tf.flush()
            os.fsync(tf.fileno())
        os.replace(temp_file, filepath)
        return True
    except Exception as e:
        if temp_file and os.path.exists(temp_file):
            try:
                os.remove(temp_file)
            except OSError:
                pass
        print(f"[PERSISTENCE] Erro ao gravar atomicamente em {filepath}: {e}")
        return False


def load_json_safe(filepath: str, default: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    Carrega arquivo JSON de forma segura com fallback para valor padrão.
    Se o arquivo não existir ou estiver corrompido, retorna uma cópia de default.
    """
    if default is None:
        default = {}
    if not filepath or not os.path.isfile(filepath):
        return dict(default) if isinstance(default, dict) else default

    try:
        with open(filepath, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        print(f"[PERSISTENCE] Erro ao carregar {filepath}: {e}")
        return dict(default) if isinstance(default, dict) else default


def save_vision_profile(filepath: str, profile_data: Dict[str, Any]) -> bool:
    """
    Salva perfil visual atomicamente em disco.
    """
    payload = dict(profile_data)
    if "timestamp" not in payload:
        payload["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return atomic_write_json(filepath, payload)


def load_vision_profile(filepath: str, default: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    Carrega perfil visual do disco com tratamento de fallback seguro.
    """
    return load_json_safe(filepath, default=default)
