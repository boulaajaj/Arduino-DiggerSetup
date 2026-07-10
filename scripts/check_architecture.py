"""Architecture fitness functions (#129) — CLI and reporting.

Usage:
  python scripts/check_architecture.py            # check the repo
  python scripts/check_architecture_selftest.py   # prove each check fires

Enforces docs/architecture/ARCHITECTURE-TARGET.md section 3 on every
sketches/<name>/src/ tree: forbidden includes, layer direction, file-size
policy (150 soft / 250 hard, allowlist in .architecture-allowlist), banned
file names, no mutable namespace-scope state in domain/, generated-file
guard, firmware-version single source of truth. Bench sketches without a
src/ tree are exempt; the suite is green until Phase D populates src/.
Exit code 1 on any failure; warnings never fail the build.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
sys.path.insert(0, str(Path(__file__).parent))
import architecture_rules as rules  # noqa: E402

# Both definition forms count: the pre-#124 macro and the current const char.
VERSION_DEFINITION_PATTERN = re.compile(
    r"^\s*(?:#\s*define\s+FIRMWARE_VERSION\b|const\s+char\s+FIRMWARE_VERSION\s*\[)",
    re.M)


def load_allowlist(repo_root: Path) -> tuple[dict[str, str], list[str]]:
    """Parse .architecture-allowlist ('path | reason' per line)."""
    allowlist: dict[str, str] = {}
    failures: list[str] = []
    listing = repo_root / ".architecture-allowlist"
    if not listing.exists():
        return allowlist, failures
    for number, raw in enumerate(listing.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        path, separator, reason = line.partition("|")
        if not separator or not reason.strip():
            failures.append(f".architecture-allowlist:{number}: entry needs "
                            f"'path | reason' — a reason is required")
            continue
        normalized = path.strip().replace("\\", "/")
        if normalized in allowlist:
            failures.append(f".architecture-allowlist:{number}: duplicate "
                            f"entry for {normalized}")
            continue
        allowlist[normalized] = reason.strip()
    return allowlist, failures


def check_repo(repo_root: Path) -> tuple[list[str], list[str], list[str]]:
    """Run every check. Returns (failures, warnings, notes)."""
    failures: list[str] = []
    warnings: list[str] = []
    notes: list[str] = []
    allowlist, allowlist_failures = load_allowlist(repo_root)
    failures += allowlist_failures

    version_definitions: list[str] = []
    for sketch_dir in sorted((repo_root / "sketches").glob("*/")):
        for source in sketch_dir.rglob("*"):
            if source.suffix in rules.SOURCE_SUFFIXES and source.is_file():
                text = source.read_text(encoding="utf-8", errors="replace")
                if VERSION_DEFINITION_PATTERN.search(text):
                    version_definitions.append(
                        source.relative_to(repo_root).as_posix())

        src_root = sketch_dir / "src"
        if not src_root.is_dir():
            continue  # bench sketch / pre-Phase-D production sketch: exempt

        # The composition-root rule (.ino -> application/ only) is a property
        # of the END state: it activates once FirmwareApp.h exists (#117 step
        # 11). During migration steps 1-10 the .ino is the interim application
        # shell and may include extracted layers directly (#150); the src/
        # files themselves are fully checked from step 1 either way.
        if (src_root / "application" / "FirmwareApp.h").is_file():
            for ino in sketch_dir.glob("*.ino"):
                failures += rules.check_ino(ino, src_root)
        else:
            notes.append(f"{sketch_dir.name}: no src/application/FirmwareApp.h"
                         " yet — composition-root .ino rule activates at Phase"
                         " D step 11 (#150)")
        for file in sorted(src_root.rglob("*")):
            if not (file.is_file() and file.suffix in rules.SOURCE_SUFFIXES):
                continue
            layer = rules.layer_of(file, src_root)
            if layer == "generated":
                continue  # machine-written: size/name/include rules don't apply
            failures += rules.check_includes(file, src_root)
            failures += rules.check_banned_name(file)
            size_failures, size_warnings = rules.check_file_size(
                file, allowlist, repo_root)
            failures += size_failures
            warnings += size_warnings
            if layer == "domain":
                failures += rules.check_domain_globals(file)

        generated = src_root / "generated"
        if generated.is_dir() and not (repo_root / "scripts" / "generate_web_page.py").exists():
            notes.append(f"{generated}: exists but no generator yet — drift "
                         f"check activates with #120")

    if len(version_definitions) > 1:
        failures.append("FIRMWARE_VERSION defined in more than one file "
                        f"(single source of truth, #124): {version_definitions}")
    elif not version_definitions:
        notes.append("no FIRMWARE_VERSION definition yet — SSOT check "
                     "activates with #124")
    return failures, warnings, notes


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    failures, warnings, notes = check_repo(repo_root)
    for note in notes:
        print(f"NOTE: {note}")
    for warning in warnings:
        print(f"WARN: {warning}")
    for failure in failures:
        print(f"FAIL: {failure}")
    if failures:
        print(f"\narchitecture-fitness: {len(failures)} violation(s)")
        return 1
    print(f"architecture-fitness: OK ({len(warnings)} warning(s))")
    return 0


if __name__ == "__main__":
    sys.exit(main())
