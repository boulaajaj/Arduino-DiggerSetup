"""Self-test for the architecture fitness functions (#129).

Builds a throwaway repo skeleton with one deliberate violation per check,
asserts each check fires, then asserts a clean skeleton passes. Run by CI
before the real check so a silently-broken checker cannot go green.
"""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from check_architecture import check_repo  # noqa: E402

EXPECTED_VIOLATIONS = {
    "hardware header": "src/domain/Speed.h includes <Arduino.h>",
    "may not include infrastructure/": "domain/ -> infrastructure/",
    "may not include domain/": "infrastructure/ -> domain/ directly",
    "may not include application/": "telemetry/ -> application/ (non-snapshot)",
    "banned file name": "src/domain/TrackUtils.h",
    "exceeds hard limit": "251-line file, not allowlisted",
    "mutable namespace-scope state": "int currentGear = 0; in domain/",
    "more than one file": "FIRMWARE_VERSION defined twice (#define + const char forms)",
    ".ino may only include application/": "sketch.ino includes domain/ directly",
    "entry needs 'path | reason'": "allowlist entry missing reason",
    "duplicate entry": "same allowlist path twice (one with backslashes)",
    "GearPolicy.cpp:5:": "line number correct after a multi-line block comment",
    "slippedThrough": "namespace with brace on the NEXT line still checked",
}


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def build_violating_repo(root: Path) -> None:
    src = root / "sketches" / "fixture" / "src"
    write(root / "sketches" / "fixture" / "fixture.ino",
          '#include "src/domain/Speed.h"\n')
    write(src / "domain" / "Speed.h",
          "#include <Arduino.h>\n"
          '#include "../infrastructure/EscPwm.h"\n'
          "#define FIRMWARE_VERSION \"1\"\n")
    write(src / "domain" / "TrackUtils.h", "// banned name\n")
    write(src / "domain" / "GearPolicy.cpp",
          "/* block\n   comment\n   lines */\n"
          "namespace gear {\nint currentGear = 0;\n"
          "constexpr float kMax = 1.0f;\n}\n"
          "namespace gear_brace_next_line\n{\nint slippedThrough = 1;\n}\n"
          "const char FIRMWARE_VERSION[] = \"2\";\n")
    write(src / "infrastructure" / "EscPwm.h",
          '#include "../domain/Speed.h"\n')
    write(src / "telemetry" / "JsonEncoder.h",
          '#include "../application/ControlCycle.h"\n')
    write(src / "application" / "ControlCycle.h", "// fine\n")
    write(src / "application" / "SystemSnapshot.h", "// fine\n")
    write(src / "domain" / "Oversize.h", "// line\n" * 251)
    write(root / ".architecture-allowlist",
          "missing/reason.h\n"
          "twice/listed.h | first reason\n"
          "twice\\listed.h | second reason\n")


def build_clean_repo(root: Path) -> None:
    src = root / "sketches" / "fixture" / "src"
    write(root / "sketches" / "fixture" / "fixture.ino",
          "#include <Arduino.h>\n"
          '#include "src/application/ControlCycle.h"\n')
    write(src / "application" / "ControlCycle.h",
          '#include "../domain/CurvatureDrive.h"\n'
          '#include "../ports/EscOutputPort.h"\n')
    write(src / "application" / "SystemSnapshot.h", "// snapshot\n")
    write(src / "domain" / "CurvatureDrive.h",
          "constexpr float kPivotCap = 0.6f;\n"
          "namespace drive {\nconst int kGears = 3;\n}\n")
    write(src / "ports" / "EscOutputPort.h",
          '#include "../domain/CurvatureDrive.h"\n')
    write(src / "infrastructure" / "EscPwm.h",
          "#include <Servo.h>\n"
          '#include "../ports/EscOutputPort.h"\n')
    write(src / "telemetry" / "JsonEncoder.h",
          '#include "../application/SystemSnapshot.h"\n')
    write(src / "domain" / "Allowlisted.h", "// line\n" * 251)
    write(root / ".architecture-allowlist",
          "# path | reason\n"
          "sketches/fixture/src/domain/Allowlisted.h | self-test fixture\n")


def main() -> int:
    with tempfile.TemporaryDirectory() as raw:
        root = Path(raw)
        build_violating_repo(root)
        failures, _, _ = check_repo(root)
        report = "\n".join(failures)
        missing = [f"self-test: expected a '{marker}' failure ({why})"
                   for marker, why in EXPECTED_VIOLATIONS.items()
                   if marker not in report]
        for problem in missing:
            print(f"FAIL: {problem}")
        if missing:
            print("\n--- violating-fixture output for debugging ---")
            print(report)
            return 1

    with tempfile.TemporaryDirectory() as raw:
        root = Path(raw)
        build_clean_repo(root)
        failures, warnings, _ = check_repo(root)
        if failures:
            print("FAIL: self-test: clean fixture should pass, got:")
            print("\n".join(failures))
            return 1

    print(f"architecture-fitness self-test: OK "
          f"({len(EXPECTED_VIOLATIONS)} failure modes demonstrated)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
