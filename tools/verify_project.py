"""Run the complete suite; fail verification if any tests fail or are skipped."""
import argparse
from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime, timezone
import importlib
from io import StringIO
import json
import os
from pathlib import Path
import platform
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def execute_suite(suite, output_dir, environment, dependency_errors=()):
    """Write machine-readable status and the unmodified test-runner output."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    log = StringIO()
    result = None
    if dependency_errors:
        log.write("Required dependencies could not be imported:\n")
        for message in dependency_errors:
            log.write(message + "\n")
        log.write("Install requirements.txt using the interpreter running this command.\n")
    else:
        with redirect_stdout(log), redirect_stderr(log):
            result = unittest.TextTestRunner(stream=log, verbosity=2).run(suite)
    passed = bool(result is not None and result.testsRun > 0 and result.wasSuccessful()
                  and not result.skipped and not result.expectedFailures)
    report = {
        "status": "passed" if passed else "failed",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "commit": os.environ.get("GITHUB_SHA"),
        "environment": environment,
        "tests_run": result.testsRun if result else 0,
        "failures": len(result.failures) if result else 0,
        "errors": len(result.errors) if result else 0,
        "skipped": len(result.skipped) if result else 0,
        "expected_failures": len(result.expectedFailures) if result else 0,
        "unexpected_successes": len(result.unexpectedSuccesses) if result else 0,
        "dependency_errors": list(dependency_errors),
        "policy": "All discovered tests must pass; missing dependencies, empty suites and skipped tests fail verification.",
    }
    if result is not None and result.skipped:
        log.write("\nVerification failed: the complete suite must run without skipped tests.\n")
    if result is not None and result.testsRun == 0:
        log.write("\nVerification failed: no tests were discovered.\n")
    (output_dir / "tests.txt").write_text(log.getvalue(), encoding="utf-8")
    (output_dir / "summary.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(log.getvalue(), end="")
    print(f"Verification: {report['status']}")
    print(f"Reports: {output_dir}")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "results" / "verification")
    args = parser.parse_args()
    environment = {"python": platform.python_version(), "platform": platform.system()}
    errors = []
    for name in ("numpy", "scipy", "matplotlib"):
        try:
            module = importlib.import_module(name)
            environment[name] = module.__version__
        except Exception as error:
            errors.append(f"{name}: {type(error).__name__}: {error}")
    suite = unittest.TestSuite() if errors else unittest.defaultTestLoader.discover(str(ROOT / "tests"))
    report = execute_suite(suite, args.output_dir, environment, errors)
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
