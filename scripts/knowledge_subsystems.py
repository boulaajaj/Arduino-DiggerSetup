"""Source-backed subsystem summaries and extracted interface contracts."""
import hashlib
import json
import re

from knowledge_catalog import line_ref

SRC = "sketches/dual_track_control/src/"
APP = SRC + "application/"
CONTROL = APP + "FirmwareApp.cpp"
MONITOR = APP + "Monitoring.cpp"
INPUT = APP + "OperatorInput.cpp"
ALERT = APP + "AlertControl.cpp"
FRONT = "dashboard/index.html"
NET = SRC + "infrastructure/network/"
XB = SRC + "infrastructure/xc/XbusTelemetryAdapter"
JENC = SRC + "telemetry/JsonEncoder.cpp"
CENC = SRC + "telemetry/CsvEncoder.cpp"
SAFE = "docs/SAFETY.md"
SCALE = SRC + "telemetry/TelemetryScaling.h"

# Explanations are reviewed descriptions. Exact interface members and selected
# firmware/browser values are extracted below, with a build failure on drift.
DETAILS = {
    "Control safeguards": [
        ("Control order", "Read RC, evaluate battery and thermal protection, select gear, sample joystick, mix inputs, and apply the output gate. Service the watchdog after output, then run telemetry, Wi-Fi, alerts and debug.", CONTROL),
        ("Receiver flags", "A frame carrying the receiver failsafe flag invalidates RC immediately. The stale timer separately invalidates RC when frames stop. The lost-frame bit is recorded for diagnostics and does not independently close the gate.", CONTROL, SRC + "infrastructure/radiolink/SbusReceiverAdapter.cpp"),
        ("Authority modes", "The mapped CH5 switch selects remote only, joystick with RC override, or shared control. In shared control, each axis uses the stronger same-direction command or combines opposing commands so they can cancel. Valid RC is required in all modes.", INPUT, SRC + "domain/drive/CommandMixer.cpp"),
        ("Output limits and gear", "PWM conversion and output both clamp commands to the servo rails. Gear and reverse limits apply to the average track command, with intentional per-track turning headroom. Forced Eco overrides the selected gear.", INPUT, APP + "MotorOutput.cpp", SRC + "domain/drive/GearPolicy.cpp"),
        ("Watchdog and bench evidence", "The hardware watchdog arms after Wi-Fi startup and is refreshed once per control pass after the gate. Host tests check the refresh contract. Physical reset and ESC response belong to bench verification.", CONTROL, SAFE, "docs/architecture/BENCH-VERIFICATION-DEFERRED.md"),
        ("Known limitations", "The safety registry records a slot-dependent NaN case in the pack plausibility helper and a short reverse-cap transient on joystick cache during upshift. The production X.BUS integer decoder cannot generate a NaN pack voltage.", SAFE, "tests/invariants/test_invariant_plausibility.cpp", "tests/invariants/test_invariant_gear_reverse_caps.cpp"),
    ],
    "Notification channels": [
        ("Piezo, ESC and dashboard", "The Arduino piezo combines the manual horn, one queued one-shot, and the selected repeating alarm. ESC loss-of-PWM beeps come from the ESCs. The browser renders its own safety banners from firmware flags and reports connection or JSON/render errors separately.", ALERT, APP + "MotorOutput.cpp", FRONT),
        ("Pattern playback", "The policy selects a repeating alarm by priority. Durations alternate ON and OFF, and a repeating sequence starts at its first phase when the selected sequence changes. A new one-shot takes over the one-shot slot. The manual horn can mask pauses in both paths.", ALERT, SRC + "alerts/PatternPlayer.cpp", SRC + "alerts/AlertPolicy.cpp"),
        ("Diagnostic notifications", "The controller prints a watchdog armed/failure message at startup, Wi-Fi status periodically, and adapter startup messages. The browser has a reconnect button and reports connection state separately from motor protection.", CONTROL, MONITOR, NET + "WifiService.cpp", FRONT),
    ],
    "Dashboard display": [
        ("Track panels", "Left and right show ESC temperature, motor temperature, measured speed and power estimated as volts × amps. Temperatures have display color bands. Each pack has independent voltage, voltage-derived charge percentage, gauge and color.", FRONT),
        ("Speed, direction and commanded throttle", "Vehicle speed estimates km/h from the average of signed track RPM, gear reduction and wheel circumference. Track arrows and vehicle heading use PWM commands, with a neutral dead zone. Bottom bars show signed left/right throttle, summed current, summed power and time since the first received browser frame.", FRONT),
        ("Gear, authority and version", "Gear badges reflect the effective firmware gear, including forced Eco. Authority pills display the telemetry mode, and the version badge is injected from BuildInfo during dashboard generation. The BRANDING block controls names and accents.", FRONT, SRC + "config/BuildInfo.h", "scripts/generate_web_page.py"),
        ("Freshness and smoothing", "A stale ESC panel dims and keeps its previous values. Measurements arriving in tenths are divided by ten. A continuous animation eases display values toward their last accepted targets. The firmware has a separate, longer X.BUS validity timer.", FRONT, XB + ".cpp"),
        ("Safety banner priority", "Thermal cut → battery cut → thermal Eco → battery Eco → thermal warning → a brief thermal-restored banner. The browser selects the hottest displayed sensor label for heat messages. Banners reflect observed flags; browser status cannot open the output gate.", FRONT, JENC),
        ("Connection and recovery", "The browser opens EventSource on the Arduino. It labels a quiet stream OFFLINE, forces reconnect when received frames stop for longer, and can reconnect manually. Parsing and rendering failures are shown in a separate connection banner.", FRONT),
    ],
    "Wi-Fi service": [
        ("AP and static page", "The Arduino hosts the read-only Wi-Fi AP and HTTP dashboard. The embedded page is served in chunks between control passes. The static page uses a content ETag and can return 304 on a matching conditional request. A client disconnect or zero-byte write aborts a page transfer.", NET + "WifiService.cpp", NET + "DashboardServer.cpp", MONITOR),
        ("Snapshot and SSE", "GET /data returns application/json and closes its socket. GET /events returns text/event-stream and keeps its socket. Both share the JSON encoder and allow cross-origin reads. SSE holds one persistent client, replaces any prior socket, sends a heartbeat comment with each data message, and closes disconnected sockets or sockets whose write returns zero.", NET + "DashboardServer.cpp", NET + "SseStream.cpp", JENC),
    ],
    "X.BUS pipeline": [
        ("Request and parser", "Serial1 sends a point-to-point register read with an outgoing checksum, alternating controllers. The receiving parser screens for slave header, target address, read function, and complete register triplets. Its acceptance check does not verify a reply checksum.", XB + ".cpp", XB + ".h", "docs/XBUS-PROTOCOL.md"),
        ("Polling and freshness", "The adapter runs across control iterations. A reply or timeout advances to the next ESC, and a good reply updates validity and last-good time. The firmware may mark one ESC stale while the other remains valid.", XB + ".cpp", XB + ".h"),
        ("Register decoding", "Voltage is unsigned in tenths of a volt, current is signed in tenths of an amp, and speed is signed electrical Hz. Temperatures use the low byte with a −40 °C offset. Voltage, current and temperatures use per-ESC exponential averages; speed is instantaneous.", XB + ".cpp", XB + ".h", SCALE),
        ("Consumers", "The local protection and alert logic use smoothed voltage and temperature. Monitoring snapshots copy inputs, commanded outputs, mode, flags and both ESC readings for JSON or CSV encoders. The browser has no motor control path.", APP + "SafetyControl.cpp", ALERT, MONITOR, APP + "SystemSnapshot.h"),
    ],
    "Serial diagnostics": [
        ("CSV output", "The USB debug observer periodically writes one comma-separated snapshot when the console is ready. It includes input pulse equivalents, joystick raw ADC, output PWM, gear, raw receiver flags and both ESC measurements. CSV carries electrical Hz rather than the dashboard’s RPM conversion.", MONITOR, CENC, SRC + "config/TelemetryConfig.h"),
        ("Bench tooling", "The repository includes serial monitoring and telemetry plotting tools, standalone sketches for decoding and checking the bus, and replay tooling and UI checks for the dashboard.", "tools/README.md", "tools/monitor.py", "tools/live_plot_telemetry.py", "sketches/README.md"),
    ],
}

EXTRA_TOPICS = {
    "safety-system": ("Control safeguards", "Safety invariants", "Protection and alert settings"),
    "output-gate": ("Control safeguards", "Safety invariants"),
    "sbus-input": ("Control safeguards", "Operator input settings"),
    "control-pipeline": ("Control safeguards", "Operator input settings", "Drive and output settings"),
    "gear-and-reverse-caps": ("Control safeguards", "Drive and output settings"),
    "override-modes": ("Control safeguards", "Operator input settings"),
    "configuration-authority": ("Operator input settings", "Drive and output settings", "Protection and alert settings", "Board and observer settings"),
    "application-loop": ("Control safeguards", "Notification channels", "X.BUS pipeline", "Serial diagnostics", "Board and observer settings"),
    "testing": ("Safety invariants",),
    "drive-safeguards": ("Control safeguards", "Safety invariants", "Drive and output settings"),
    "notification-channels": ("Notification channels", "Dashboard display"),
    "dashboard-display": ("Dashboard display", "Dashboard settings", "Dashboard elements"),
    "telemetry-interfaces": ("Wi-Fi service", "JSON fields", "Serial diagnostics", "Serial CSV fields"),
    "alerts-and-alarms": ("Notification channels", "Dashboard display", "Protection and alert settings"),
    "telemetry-and-dashboard": ("Dashboard display", "Wi-Fi service", "JSON fields", "X.BUS pipeline", "Serial diagnostics", "Serial CSV fields"),
    "wifi-dashboard": ("Dashboard display", "Dashboard settings", "Dashboard elements", "Wi-Fi service", "JSON fields"),
    "xbus-telemetry": ("X.BUS pipeline", "X.BUS settings and registers", "JSON fields", "Serial diagnostics", "Serial CSV fields"),
}

JSON_MEANINGS = {
    "t": "Controller time in ms at observation.", "seq": "Sequence incremented for each built JSON frame, shared by snapshot and stream.",
    "gear": "Effective gear enum: 0 Eco, 1 Normal, 2 Boost.", "mode": "Authority: 0 RC, 1 joystick/RC override, 2 shared control.",
    "fs": "Raw receiver failsafe flag (0/1). This is distinct from the internal RC stale timer.", "lost": "Raw lost-frame flag (0/1).",
    "eco": "Latched low-voltage Eco stage (0/1).", "cut": "Latched low-voltage drive cutoff (0/1).",
    "hot": "Thermal warning stage (0/1).", "teco": "Thermal Eco stage (0/1).", "tcut": "Thermal drive cutoff (0/1).",
    "outL": "Last written left ESC command in µs.", "outR": "Last written right ESC command in µs.",
    "ok": "Firmware validity flag (0/1).", "age": "Milliseconds since the last accepted reply; 999999 when no good sample has been received.",
    "rpm": "Signed electrical Hz × 30 (dashboard RPM).", "cur": "Signed bus current, rounded to tenths of an amp.",
    "v": "Pack voltage, rounded to tenths of a volt.", "tE": "ESC temperature, rounded to whole °C.", "tM": "Motor temperature, rounded to whole °C.",
}
CONSTANT = re.compile(
    r"^\s*(?:(?:inline|static)\s+)*(?:const|constexpr)\s+[\w:]+\s+"
    r"(?P<name>[A-Z][A-Z0-9_]+)(?:\[\])?\s*=\s*(?P<value>[^;]+);[ \t]*(?://[ \t]*(?P<comment>[^\n]*))?",
    re.M,
)
CSV_MEMBER = re.compile(r"snapshot\.(\w+(?:\[\d+\]\.\w+)?)")


def read_subsystems(root, files):
    texts = {}

    def read(path):
        if path not in files:
            raise ValueError(f"Missing tracked subsystem source: {path}")
        if path not in texts:
            texts[path] = (root / path).read_text(encoding="utf-8")
        return texts[path]

    def card(label, description, *paths):
        refs = []
        for source in paths:
            path, line = source if isinstance(source, tuple) else (source, None)
            read(path)
            refs.append({"path": path, **({"line": line} if line else {})})
        return {"label": label, "description": description, "sources": refs}

    cards = {group: [card(*row) for row in rows] for group, rows in DETAILS.items()}

    # Parse the formatted JSON object itself, so identical leaf names in
    # different objects never hide an added, removed or moved field.
    source = read(JENC)
    fmt = re.search(r"snprintf\(body,\s*capacity,\s*((?:\s*\"(?:\\.|[^\"\\])*\"\s*)+),", source)
    if not fmt:
        raise ValueError("Could not read dashboard JSON format in " + JENC)
    strings = re.findall(r'"(?:\\.|[^"\\])*"', fmt[1])
    template = "".join(json.loads(value) for value in strings)
    shape = json.loads(re.sub(r"%l?[udi]", "0", template))
    if not isinstance(shape, dict):
        raise ValueError("Dashboard JSON frame is not an object")
    json_fields = []
    for name, value in shape.items():
        if isinstance(value, dict):
            if name not in {"e0", "e1"} or set(value) != {"ok", "age", "rpm", "cur", "v", "tE", "tM"}:
                raise ValueError(f"Review dashboard ESC field descriptions for {name}")
            for key in value:
                token = re.search(r'\\"' + re.escape(key) + r'\\":', source[fmt.start():fmt.end()])
                # The format source lists each ESC object on its own line today.
                esc = re.search(r'\\"' + re.escape(name) + r'\\":', source)
                if not token or not esc:
                    raise ValueError(f"Could not locate dashboard field {name}.{key}")
                json_fields.append(card(f"{name}.{key}", JSON_MEANINGS[key], (JENC, line_ref(esc, source))))
        else:
            if name not in JSON_MEANINGS:
                raise ValueError(f"Undocumented dashboard JSON field: {name}")
            key = re.search(r'\\"' + re.escape(name) + r'\\":', source)
            json_fields.append(card(name, JSON_MEANINGS[name], (JENC, line_ref(key, source))))
    required = (set(JSON_MEANINGS) - {"ok", "age", "rpm", "cur", "v", "tE", "tM"}) | {"e0", "e1"}
    if set(shape) != required:
        raise ValueError("Dashboard JSON shape changed. Update inspector field descriptions.")
    cards["JSON fields"] = json_fields

    source = read(CENC)
    pattern = re.search(r'snprintf\(line,\s*capacity,\s*"([%dlui,]+)",([\s\S]+?)\);', source)
    members = list(CSV_MEMBER.finditer(pattern[2])) if pattern else []
    if not members or len(members) != len(pattern[1].split(",")):
        raise ValueError("CSV format no longer maps one snapshot member to each column: " + CENC)
    csv_fields = []
    for index, member in enumerate(members, 1):
        name = member[1]
        field = name.split(".")[-1]
        units = {
            "voltage": "tenths of a volt", "busCurrentA": "signed tenths of an amp",
            "rpmHz": "signed electrical Hz", "escTempC": "whole °C", "motorTempC": "whole °C",
            "valid": "0/1", "gear": "0 Eco / 1 Normal / 2 Boost",
            "rcFailsafe": "raw receiver flag, 0/1", "rcLostFrame": "raw receiver flag, 0/1",
            "joystickRawY": "raw throttle ADC", "joystickRawX": "raw steering ADC",
        }.get(field, "µs" if field.endswith("Us") else None)
        if units is None:
            raise ValueError(f"Undocumented serial CSV field: {name}")
        line = source.count("\n", 0, pattern.start(2) + member.start()) + 1
        csv_fields.append(card(f"{index}. {name}", units, (CENC, line)))
    cards["Serial CSV fields"] = csv_fields

    source = read(SAFE)
    invariants = []
    for match in re.finditer(r"^\|\s*(\d+)\s*\|\s*([^|]+)\|[^|]*\|\s*([^|\n]+)\|", source, re.M):
        tests = re.findall(r"tests/[\w/]+\.cpp", match[3])
        if not tests:
            raise ValueError(f"No test references for invariant {match[1]}")
        invariants.append(card(f"Invariant {match[1]}", match[2].strip(), (SAFE, line_ref(match, source)), *tests))
    if not invariants:
        raise ValueError("No safety invariants found in " + SAFE)
    cards["Safety invariants"] = invariants

    def settings(paths):
        rows = []
        for path in paths:
            source = read(path)
            entries = list(CONSTANT.finditer(source))
            if not entries:
                raise ValueError("No settings found in " + path)
            for entry in entries:
                name = entry["name"]
                if any(word in name for word in ("SECRET", "PASSWORD", "WIFI_PASS", "WIFI_SSID")):
                    continue
                value = " ".join(entry["value"].split())
                comment = entry["comment"]
                rows.append(card(name, f"{value} — {comment.strip()}" if comment else value,
                                 (path, line_ref(entry, source))))
        return rows

    for group, names, extra in (
        ("Operator input settings", ("InputConfig.h",), [APP + "OperatorInput.h"]),
        ("Drive and output settings", ("DriveConfig.h", "Pins.h"), []),
        ("Protection and alert settings", ("SafetyConfig.h", "BatteryConfig.h", "ThermalConfig.h", "AlertConfig.h"), []),
        ("Board and observer settings", ("TelemetryConfig.h", "WifiConfig.h", "BuildInfo.h"), [NET + "WifiService.h"]),
    ):
        cards[group] = settings([SRC + "config/" + name for name in names] + extra)
    cards["X.BUS settings and registers"] = settings([XB + ".h"])

    source = read(FRONT)
    controls = []
    values = re.finditer(r"\b([A-Z][A-Z0-9_]+)\s*=\s*('[^'\n]+'|\d+(?:\.\d+)?)(?=\s*[,;])", source)
    for value in values:
        controls.append(card(value[1], value[2], (FRONT, line_ref(value, source))))
    for lookup in ("STALE_MS", "ESC_STALE_AGE_MS", "WATCHDOG_MS", "RECONNECT_DELAY_MS", "SMOOTH", "MR", "GR", "WC", "PWR_MAX"):
        if not any(row["label"] == lookup for row in controls):
            raise ValueError(f"Missing dashboard setting: {lookup}")
    curve = re.search(r"const C\s*=\s*(\[\[.+?\]\]);", source)
    if not curve:
        raise ValueError("Missing dashboard charge curve")
    pairs = json.loads(curve[1])
    controls.append(card("Battery charge curve (display)", " · ".join(f"{volts} V → {pct}%" for volts, pct in pairs),
                         (FRONT, line_ref(curve, source))))
    cards["Dashboard settings"] = controls

    elements = []
    for match in re.finditer(r'<([a-zA-Z][\w-]*)\b[^<>]*?\bid="([\w-]+)"', source):
        elements.append(card(match[2], f"Static {match[1]} element.", (FRONT, line_ref(match, source))))
    for match in re.finditer(r"\.id\s*=\s*'([\w-]+)'", source):
        elements.append(card(match[1], "Element created by the dashboard script.", (FRONT, line_ref(match, source))))
    for match in re.finditer(r"^(?:async )?function (\w+)\(", source, re.M):
        elements.append(card(match[1], "Dashboard JavaScript function.", (FRONT, line_ref(match, source))))
    if not elements:
        raise ValueError("Could not extract dashboard elements")
    cards["Dashboard elements"] = sorted(elements, key=lambda item: item["sources"][0]["line"])

    return cards, {path: hashlib.sha256(text.encode()).hexdigest() for path, text in sorted(texts.items())}
