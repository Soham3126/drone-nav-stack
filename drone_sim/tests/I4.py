#!/usr/bin/env python3
"""I4: same procedure and checks as I2 (see I2.py); worlds and counts: scripts/run_i.sh."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from I2 import main  # noqa: E402

if __name__ == "__main__":
    main("I4")
