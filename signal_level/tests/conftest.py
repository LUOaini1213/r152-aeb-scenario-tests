import sys
from pathlib import Path

sys.dont_write_bytecode = True                                  # never write __pycache__ into the repo
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # repo root: makes aebtest and signal_level importable
