"""Locate optional CAD inputs independently of the process working directory.

HEAVY_ARM_CAD_DIR may point to the directory containing the SLDASM and its parts.
The default matches the local workspace: <workspace>/27sw/27重装11.3.
"""
from pathlib import Path
import os

CAD_DIR = Path(os.environ.get(
    'HEAVY_ARM_CAD_DIR',
    str(Path(__file__).resolve().parents[4] / '27sw' / '27重装11.3'),
)).expanduser().resolve()
ASSEMBLY_PATH = CAD_DIR / '27重装-机械臂-11.3.SLDASM'
