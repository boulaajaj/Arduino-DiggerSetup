"""Generate the embedded dashboard header from its source (#120).

dashboard/index.html is the single source of truth; this script wraps it
verbatim into sketches/dual_track_control/web_page.h as the PROGMEM raw-string literal
the firmware serves at "/". Never edit web_page.h by hand.

Usage:
  python scripts/generate_web_page.py          # regenerate web_page.h
  python scripts/generate_web_page.py --check  # exit 1 if web_page.h differs
                                               # from what the source produces

Newlines are normalized to LF so output is byte-identical on every platform
(the CI drift check depends on that). The served page must stay within the
100 KB budget (epic #81) — generation fails if the source exceeds it.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

PAGE_BUDGET_BYTES = 100 * 1024  # epic #81: served dashboard <= 100 KB
RAW_DELIMITER = "DIGGER"

HEADER = f"""\
// GENERATED FILE — do not edit. Source: dashboard/index.html
// Regenerate: python scripts/generate_web_page.py   (CI enforces sync, #120)
const char INDEX_HTML[] PROGMEM = R"{RAW_DELIMITER}(
"""
TRAILER = f"){RAW_DELIMITER}\";\n"


def generate(repo_root: Path) -> bytes:
    source_path = repo_root / "dashboard" / "index.html"
    page = source_path.read_bytes().replace(b"\r\n", b"\n")
    if len(page) > PAGE_BUDGET_BYTES:
        raise SystemExit(f"FAIL: {source_path} is {len(page)} bytes — over the "
                         f"{PAGE_BUDGET_BYTES} byte page budget (epic #81)")
    closer = f"){RAW_DELIMITER}".encode()
    if closer in page:
        raise SystemExit(f"FAIL: {source_path} contains {closer!r}, which "
                         f"terminates the raw-string literal early")
    if not page.endswith(b"\n"):
        page += b"\n"
    return HEADER.encode() + page + TRAILER.encode()


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    target = repo_root / "sketches" / "dual_track_control" / "web_page.h"
    generated = generate(repo_root)
    if "--check" in sys.argv:
        current = target.read_bytes().replace(b"\r\n", b"\n")
        if current != generated:
            print(f"FAIL: {target} is out of sync with dashboard/index.html "
                  f"— run: python scripts/generate_web_page.py")
            return 1
        print("web_page.h: in sync with dashboard/index.html")
        return 0
    target.write_bytes(generated)
    print(f"wrote {target} ({len(generated)} bytes, "
          f"page {len(generated) - len(HEADER) - len(TRAILER)} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
