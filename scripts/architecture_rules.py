"""Architecture fitness rules (#129) — pure check logic, Python stdlib only.

Encodes docs/architecture/ARCHITECTURE-TARGET.md section 3 (the dependency
table is canonical; if this file and the table disagree, the table wins).
Checks target sketches/<name>/src/ trees only — bench sketches without a
src/ tree are exempt by design, so the suite is green until Phase D
populates the structure.
"""
from __future__ import annotations

import re
from pathlib import Path

SOFT_LINE_LIMIT = 150
HARD_LINE_LIMIT = 250

SOURCE_SUFFIXES = {".h", ".hpp", ".cpp", ".ino"}

# Hardware/system headers allowed ONLY in infrastructure/ (table row 4).
HARDWARE_HEADERS = {
    "Arduino.h", "WiFiS3.h", "WiFi.h", "WiFiServer.h", "WiFiClient.h",
    "Servo.h", "sbus.h", "SBUS.h", "Wire.h", "SPI.h", "EEPROM.h",
    "SoftwareSerial.h", "WDT.h", "analogWave.h", "IRQManager.h",
}

# Banned file-name stems (.claude/rules/architecture.md). A stem equal to or
# ending with one of these fails (PwmManager, EscData, Utils, ...).
BANNED_STEMS = ("utils", "helpers", "helper", "manager", "common", "misc",
                "stuff", "data")

# Layer -> layers it may include (ARCHITECTURE-TARGET.md section 3).
# telemetry/ and alerts/ are the application observer ring: domain types and
# config only, plus the single application/SystemSnapshot.h exception.
ALLOWED_INCLUDE_LAYERS = {
    "domain":         {"domain", "config"},
    "ports":          {"ports", "domain"},
    "application":    {"application", "domain", "ports", "config",
                       "telemetry", "alerts"},
    "telemetry":      {"telemetry", "domain", "config"},
    "alerts":         {"alerts", "domain", "config"},
    "infrastructure": {"infrastructure", "ports", "config"},
    "config":         {"config"},
}
OBSERVER_LAYERS = {"telemetry", "alerts"}
OBSERVER_APPLICATION_EXCEPTION = "application/SystemSnapshot.h"

INCLUDE_PATTERN = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]', re.M)


def strip_comments(text: str) -> str:
    """Remove comments, preserving newlines so line numbers stay accurate."""
    text = re.sub(r"/\*.*?\*/",
                  lambda match: "\n" * match.group(0).count("\n"),
                  text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def layer_of(path: Path, src_root: Path) -> str:
    return path.relative_to(src_root).parts[0]


def resolve_include(file: Path, include: str, src_root: Path) -> str | None:
    """Return 'layer/…' path inside src/, or None if external to src/."""
    for base in (file.parent, src_root):
        candidate = (base / include).resolve()
        try:
            return candidate.relative_to(src_root.resolve()).as_posix()
        except ValueError:
            continue
    return None


def check_includes(file: Path, src_root: Path) -> list[str]:
    """Forbidden includes + layer direction, one file at a time."""
    violations = []
    layer = layer_of(file, src_root)
    allowed = ALLOWED_INCLUDE_LAYERS.get(layer)
    if allowed is None:  # generated/ and unknown folders: not include-checked
        return violations
    for include in INCLUDE_PATTERN.findall(strip_comments(file.read_text(encoding="utf-8", errors="replace"))):
        base = include.split("/")[-1]
        if base in HARDWARE_HEADERS:
            if layer != "infrastructure":
                violations.append(
                    f"{file}: {layer}/ must not include hardware header <{include}>")
            continue
        internal = resolve_include(file, include, src_root)
        if internal is None:
            continue  # external/library/std header — fine outside the hardware list
        target_layer = internal.split("/")[0]
        if (layer in OBSERVER_LAYERS and target_layer == "application"
                and internal == OBSERVER_APPLICATION_EXCEPTION):
            continue
        if target_layer not in allowed:
            violations.append(
                f"{file}: {layer}/ may not include {target_layer}/ ({include})")
    return violations


def check_ino(ino: Path, src_root: Path) -> list[str]:
    """.ino may include application/ only (Arduino.h is implicit and allowed)."""
    violations = []
    for include in INCLUDE_PATTERN.findall(strip_comments(ino.read_text(encoding="utf-8", errors="replace"))):
        if include.split("/")[-1] == "Arduino.h":
            continue
        internal = resolve_include(ino, include, src_root)
        if internal is None:
            violations.append(f"{ino}: .ino may only include application/ ({include})")
        elif internal.split("/")[0] != "application":
            violations.append(f"{ino}: .ino may only include application/ ({include})")
    return violations


def check_file_size(file: Path, allowlist: dict[str, str],
                    repo_root: Path) -> tuple[list[str], list[str]]:
    """Return (failures, warnings) for the 150-soft / 250-hard policy."""
    lines = len(file.read_text(encoding="utf-8", errors="replace").splitlines())
    relative = file.relative_to(repo_root).as_posix()
    if relative in allowlist:
        return [], []
    if lines > HARD_LINE_LIMIT:
        return [f"{relative}: {lines} lines exceeds hard limit {HARD_LINE_LIMIT} "
                f"(split by responsibility, or allowlist with a reason)"], []
    if lines > SOFT_LINE_LIMIT:
        return [], [f"{relative}: {lines} lines exceeds soft limit {SOFT_LINE_LIMIT}"]
    return [], []


def check_banned_name(file: Path) -> list[str]:
    stem = file.stem.lower()
    for banned in BANNED_STEMS:
        if stem == banned or stem.endswith(banned):
            return [f"{file}: banned file name (…{banned!r}) — name one precise "
                    f"responsibility (.claude/rules/architecture.md)"]
    return []


DEFINITION_PATTERN = re.compile(
    r"^[A-Za-z_][\w:<>,\s\*&]*?\s[A-Za-z_]\w*\s*(=[^;]*)?;\s*$")
SKIP_KEYWORDS = ("const ", "constexpr", "using ", "typedef ", "extern ",
                 "static_assert", "friend ", "template", "return ",
                 "struct ", "class ", "enum ", "namespace ")


def check_domain_globals(file: Path) -> list[str]:
    """Heuristic: no mutable namespace-scope variable definitions in domain/.

    Tracks brace depth; namespace blocks are scope-transparent (their contents
    are still namespace scope). Contents of any other braced scope (function,
    class, initializer) are skipped. const/constexpr definitions pass.
    Each '{' is classified by the text since the previous delimiter, so
    'namespace foo' with the brace on the NEXT line still counts as namespace.
    """
    violations = []
    scopes: list[str] = []  # stack of "ns" | "other"
    context = ""  # accumulated text since the last '{' / '}' / ';'
    for number, raw in enumerate(
            strip_comments(file.read_text(encoding="utf-8", errors="replace")).splitlines(), 1):
        line = raw.strip()
        at_namespace_scope = all(scope == "ns" for scope in scopes)
        if (at_namespace_scope and line and not line.startswith("#")
                and "(" not in line and "{" not in line and "}" not in line
                and not any(line.startswith(k) or f" {k}" in f" {line}" for k in SKIP_KEYWORDS[:2])
                and not any(line.startswith(k) for k in SKIP_KEYWORDS)
                and DEFINITION_PATTERN.match(line)):
            violations.append(
                f"{file}:{number}: mutable namespace-scope state in domain/ "
                f"({line[:60]}) — modules own state behind functions")
        for char in raw:
            if char == "{":
                scopes.append(
                    "ns" if context.strip().startswith("namespace") else "other")
                context = ""
            elif char == "}":
                if scopes:
                    scopes.pop()
                context = ""
            elif char == ";":
                context = ""
            else:
                context += char
        context += " "  # newline: keep 'namespace foo' linked to next-line '{'
    return violations
