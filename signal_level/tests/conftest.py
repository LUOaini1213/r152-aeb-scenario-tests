import sys
from pathlib import Path

sys.dont_write_bytecode = True                                  # never write __pycache__ into P5's folders
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # P5 root: makes aebtest and signal_level importable
