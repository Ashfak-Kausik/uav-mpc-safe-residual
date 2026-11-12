"""Run every check_*.py in this folder and summarise. Usage: python run_all.py

A FAIL is a finding about the notebook, not necessarily a bug in the script:
each FAIL line says which claim in which entry did not hold.
"""
import pathlib
import subprocess
import sys

here = pathlib.Path(__file__).parent
failed = []
for script in sorted(here.glob("check_*.py")):
    print(f"===== {script.name}")
    r = subprocess.run([sys.executable, script.name], cwd=here)
    if r.returncode:
        failed.append((script.name, r.returncode))
print("\nSummary:", "all checks passed" if not failed else ", ".join(f"{n}: {k} FAIL" for n, k in failed))
sys.exit(1 if failed else 0)
