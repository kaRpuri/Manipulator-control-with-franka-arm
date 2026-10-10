"""Put the library packages in src/ on sys.path.

Entry points in this directory import this module first so that
``python scripts/<name>.py`` works from any working directory without
requiring PYTHONPATH to be set by hand.
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
MAPS = REPO_ROOT / "maps"

if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
