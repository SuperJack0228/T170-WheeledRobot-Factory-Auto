#!/usr/bin/env python3
"""Offline dynamic head-to-base transform CLI; does not open robot hardware."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"python"))
from t170_vision.calibration.head_transform import main
if __name__ == "__main__": raise SystemExit(main())
