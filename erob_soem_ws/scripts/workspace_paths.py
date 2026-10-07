"""Shared output paths for workspace acceptance scripts."""

import os
from pathlib import Path


def test_log_directory(name):
    workspace = Path(__file__).resolve().parents[1]
    profile = 'dev' if os.environ.get('CONDA_PREFIX') else 'native'
    root = Path(os.environ.get('EROB_TEST_LOG_DIR', workspace / 'log' / profile / 'tests'))
    logs = root.resolve() / name
    logs.mkdir(parents=True, exist_ok=True)
    return logs
