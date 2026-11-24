"""Extract supported wiring paths and link tests to the code they check."""
from functools import lru_cache
import json
import re

SRC = "sketches/dual_track_control/src/"
GUIDE = "docs/WIRING-GUIDE-V8.md"
NETLIST = "docs/INTERFACE-BOARD-NETLIST.md"
HUB = "docs/wiki/hardware.md"
WIRING = "docs/wiki/wiring-and-ports.md"
TESTING = "docs/wiki/testing.md"
COMPONENTS = {
    "left-esc": ("Left GL10 ESC", "Left throttle, shared X.BUS, and the documented BEC feed."),
    "right-esc": ("Right GL10 ESC", "Right throttle and shared X.BUS telemetry."),
    "receiver": ("Radiolink receiver", "S.BUS receiver connected through the interface board inverter."),
    "joystick": ("Hall-effect joystick", "Two analog axes for throttle and steering."),
    "board": ("Interface board", "Combines X.BUS signals and inverts the receiver's S.BUS signal."),
    "piezo": ("Active piezo", "Audible horn and controller alert patterns."),
    "usb-host": ("USB host", "Computer connected for serial debug and uploads."),
}
# (component, direction out of Arduino, additional route context)
PIN_PATHS = {
    "D0": ("board", False, "Both ESCs share the X.BUS node; J4 carries the receive signal."),
    "D1": ("board", True, "The transmit signal passes through the board's series resistor to the shared X.BUS node."),
    "D9": ("left-esc", True, ""), "D10": ("right-esc", True, ""),
    "D12": ("receiver", False, "Receiver → interface board J5 → Q1 inverter → J6 → Arduino D12."),
    "D8": ("piezo", True, ""), "A0": ("joystick", False, ""), "A1": ("joystick", False, ""),
    "VIN": ("left-esc", False, "The wiring guide identifies the left ESC BEC as the VIN source."),
    "5V": ("board", True, ""), "GND": ("board", None, "Shared ground across connected devices."),
}
PIN_DEFINITIONS = {
    "D8": ("config/Pins.h", "PIN_BEEPER"), "D9": ("config/Pins.h", "PIN_ESC_L"),
    "D10": ("config/Pins.h", "PIN_ESC_R"),
    "D12": ("infrastructure/radiolink/SbusReceiverAdapter.cpp", "PIN_SBUS_RX"),
    "D11": ("infrastructure/radiolink/SbusReceiverAdapter.cpp", "PIN_SBUS_TX"),
    "A0": ("application/OperatorInput.h", "PIN_JOY_Y"), "A1": ("application/OperatorInput.h", "PIN_JOY_X"),
}
# End-to-end or UI tests whose fixtures reach the implementation indirectly.
COVERAGE = {
    "tests/invariants/test_invariant_stale_rc_neutral.cpp": [
        ("domain/safety/OutputGate.cpp", "RC loss ramps to neutral and detaches PWM", "RC silence mid-drive"),
        ("application/FirmwareApp.cpp", "Receiver failsafe and fresh-frame recovery", "recovery after RC loss"),
        ("domain/safety/SafetySupervisor.cpp", "Missing RC blocks propulsion in all modes", "dead RC + railed joystick"),
    ],
    "tests/invariants/test_invariant_cutoff_dominates.cpp": [
        ("domain/battery/BatteryLadder.cpp", "Battery cutoff remains latched after voltage recovers", "recovery voltage does not unlatch"),
        ("domain/safety/OutputGate.cpp", "The gate reaches neutral before detaching", "gate ramp"),
    ],
    "tests/invariants/test_invariant_thermal_monotone.cpp": [
        ("domain/thermal/ThermalDerating.cpp", "Thermal stages and debounce", "thermal derating sweep"),
        ("domain/thermal/ThermalHysteresis.cpp", "Cooldown and release hysteresis", "thermal hysteresis release"),
    ],
    "tests/invariants/test_invariant_gear_reverse_caps.cpp": [
        ("domain/drive/GearPolicy.cpp", "Average gear and reverse caps across control modes", "steady-state sweep"),
    ],
    "tests/invariants/test_invariant_output_bounds.cpp": [
        ("application/MotorOutput.cpp", "Hardware output clamps arbitrary integers", "hardware boundary clamps arbitrary integers"),
    ],
    "tests/invariants/test_invariant_plausibility.cpp": [
        ("domain/battery/VoltagePlausibility.cpp", "Implausible or missing pack readings", "implausible or missing pack readings"),
    ],
    "tools/ui-test-safety.mjs": [("/dashboard/index.html", "Dashboard safety banners and recovery", "safetyBanner")],
    "tools/ui-test-thermal.mjs": [("/dashboard/index.html", "Dashboard hot, cutoff, and restored banners", "applyData")],
}


def add_relationships(root, nodes, edges):
    @lru_cache(maxsize=None)
    def read(path):
        return (root / path).read_text(encoding="utf-8")

    def evidence(path, token):
        for line, text in enumerate(read(path).splitlines(), 1):
            if token in text:
                return {"path": path, "line": line}
        raise ValueError(f"Relationship evidence missing: {path}: {token}")

    def entity(key, name, role, summary):
        path = "wiring:" + key
        nodes[path] = {"id": path, "title": name, "kind": "entity", "summary": summary,
                       "branchParent": WIRING, "branchRole": role, "domain": HUB}
        return path

    def edge(a, b, kind, label, *sources, **details):
        if a not in nodes or b not in nodes:
            raise ValueError(f"Unknown relationship endpoint: {a} → {b}")
        edges.append({"from": a, "to": b, "type": kind, "label": label,
                      "evidence": list(sources), **details})

    def wire(a, b, label, *sources, **details):
        edge(a, b, "wiring", label, *sources, **details)

    def rows(section, columns):
        active = False
        for number, line in enumerate(read(GUIDE).splitlines(), 1):
            if line.startswith("#"):
                if active:
                    return
                active = section in line
            if active and line.startswith("|"):
                values = [value.strip().replace("**", "") for value in line.strip("|").split("|")]
                if len(values) == columns and not values[0].startswith("-"):
                    yield number, values

    for topic in (HUB, WIRING):
        nodes[topic]["wiringIndex"] = True
    for key, (name, summary) in COMPONENTS.items():
        entity(key, name, "Component", summary)
    pins = {}
    for line, (pin, signal, direction, port, route) in rows("Pin Assignments", 5):
        if pin == "Pin":
            continue
        if pin not in PIN_PATHS or pin in pins:
            raise ValueError(f"Review the documented wiring for {pin}")
        refs = [{"path": GUIDE, "line": line}]
        if pin in PIN_DEFINITIONS:
            file, symbol = PIN_DEFINITIONS[pin]
            suffix = pin.removeprefix("D")
            assignment = re.search(rf"\b{symbol}\s*=\s*{suffix}\s*;", read(SRC + file))
            if not assignment:
                raise ValueError(f"Wiring guide and firmware disagree for {symbol}: expected {pin}")
            refs.append({"path": SRC + file, "line": read(SRC + file).count("\n", 0, assignment.start()) + 1})
        pins[pin] = entity(pin, f"Arduino {pin}", "Arduino pin", f"{signal}. {direction}. {route}.")
        component, outward, extra = PIN_PATHS[pin]
        ends = ("wiring:" + component, pins[pin])
        if outward:
            ends = ends[::-1]
        wire(*ends, f"{signal} · {direction} · {route}", *refs,
             note=extra, direction=outward is not None, pinIndex=True)
    if set(pins) != set(PIN_PATHS):
        raise ValueError("Wiring map is missing documented pin assignments")

    for left, pin, label, token in [
        ("left-esc", "board", "Left ESC X.BUS connector → shared bus at J1", "J1.SIG, J2.SIG"),
        ("right-esc", "board", "Right ESC X.BUS connector → shared bus at J2", "J1.SIG, J2.SIG"),
        ("receiver", "board", "Receiver plugs into J5; Q1 inverts S.BUS before J6", "Receiver plugs into J5"),
    ]:
        wire("wiring:" + left, "wiring:" + pin, label,
             evidence(GUIDE if left == "receiver" else NETLIST, token), direction=left == "receiver")

    usb = entity("USB-C", "Arduino USB-C", "Arduino port", "USB connection for debug serial and uploads.")
    reserved = entity("D11", "Arduino D11", "Arduino pin", "SCI0 TX is reserved as the S.BUS UART partner; the receiver does not use it.")
    receiver_adapter = SRC + PIN_DEFINITIONS["D11"][0]
    if not re.search(r"\bPIN_SBUS_TX\s*=\s*11\s*;", read(receiver_adapter)):
        raise ValueError("Review reserved S.BUS UART TX pin")
    for line, (port, sci, _, baud, purpose) in rows("UART Architecture", 5):
        refs = [{"path": GUIDE, "line": line}]
        if port == "Serial1":
            serial = entity("Serial1", f"Serial1 ({sci})", "Arduino port", f"{purpose}. Documented baud: {baud}.")
            wire(pins["D0"], serial, "RX pin for X.BUS telemetry", *refs)
            wire(serial, pins["D1"], "TX pin for X.BUS telemetry", *refs)
        elif port == "sbusUart":
            serial = entity("sbusUart", f"sbusUart ({sci})", "Arduino port", f"{purpose}. Documented UART format: {baud}.")
            wire(pins["D12"], serial, "RX pin for S.BUS", *refs)
            wire(serial, reserved, "Reserved TX partner; no receiver wire", *refs)
        elif port == "Serial":
            wire("wiring:usb-host", usb, "USB serial debug and uploads", *refs, pinIndex=True)
    if not all(key in nodes for key in ("wiring:Serial1", "wiring:sbusUart")):
        raise ValueError("Wiring map is missing a documented UART")

    # Match a direct test invocation against a function declared in an
    # included header. A fixture-only include stays a dependency.
    tests = set(COVERAGE)
    for item in list(edges):
        a, b = item["from"], item["to"]
        if item["type"] != "includes" or not a.startswith("tests/") or not b.startswith(SRC):
            continue
        sibling = b.removesuffix(".h") + ".cpp"
        target = sibling if sibling in nodes else b
        if not a.rsplit("/", 1)[-1].startswith("test_") or "TEST_CASE(" not in read(a):
            continue
        names = {e["label"] for e in nodes[b].get("codeEntries", []) if e["kind"] == "Function"}
        calls = []
        for line, text in enumerate(read(a).splitlines(), 1):
            if text.lstrip().startswith(("//", "#")):
                continue
            invoked = {name for name in names if re.search(r"\b" + re.escape(name) + r"\s*\(", text)}
            calls.extend((name, line) for name in sorted(invoked))
        if calls:
            name, line = calls[0]
            symbols = ", ".join(sorted({symbol for symbol, _ in calls}))
            tests.add(a)
            edge(a, target, "coverage", f"Test invokes {symbols}", {"path": a, "line": line})

    for test, cases in COVERAGE.items():
        for target, label, token in cases:
            edge(test, target.lstrip("/") if target.startswith("/") else SRC + target,
                 "coverage", label, evidence(test, token))

    for path in sorted(tests):
        nodes[path].setdefault("branchParent", TESTING)
        nodes[path].setdefault("branchRole", "Test")

    # External dependencies have evidence but no repository file of their own.
    for name, adapter, token in (
        ("Servo", "infrastructure/arduino/PwmEscAdapter.cpp", "<Servo.h>"),
        ("Watchdog", "infrastructure/arduino/WatchdogAdapter.cpp", "<WDT.h>"),
        ("S.BUS library", "infrastructure/radiolink/SbusReceiverAdapter.cpp", '"sbus.h"'),
        ("WiFiS3", "infrastructure/network/WifiService.h", "<WiFiS3.h>"),
    ):
        key = "dependency:" + token.strip('<>"')
        nodes[key] = {"id": key, "title": name, "kind": "entity", "branchRole": "Library",
                      "branchParent": "docs/wiki/ports-and-adapters.md",
                      "summary": f"Firmware library included through {token}."}
        path = SRC + adapter
        edge(path, key, "dependency", "Includes " + token, evidence(path, token))

    manifest = "package.json"
    playwright = json.loads(read(manifest))["devDependencies"]["playwright"]
    key = "dependency:playwright"
    nodes[key] = {"id": key, "title": "Playwright", "kind": "entity", "branchRole": "Library",
                  "branchParent": TESTING, "summary": f"Dashboard browser test dependency ({playwright})."}
    for path in sorted(p for p in nodes if p.startswith("tools/ui-test") and p.endswith(".mjs")):
        source = next((line for line in read(path).splitlines() if "from 'playwright'" in line), None)
        if source:
            edge(path, key, "dependency", "Imports Playwright to run browser checks",
                 evidence(path, source), evidence(manifest, '"playwright"'))
