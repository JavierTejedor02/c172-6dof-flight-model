# Verification summary

## Latest full-dependency execution

- Date: 2026-11-13.
- Software release: 0.5.0.
- Result: 42 tests passed; zero failures, errors or skips.
- Environment: project virtual environment on Windows, with SciPy installed.

Command, executed from the repository root:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

## Preserved earlier execution

[verification.txt](verification.txt) records an earlier execution without SciPy: 42 tests discovered, 34 passed and eight skipped.

Passing the suite provides numerical and implementation verification. It does not establish agreement with measured aircraft performance or fuel consumption.
