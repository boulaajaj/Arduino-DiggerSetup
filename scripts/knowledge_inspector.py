"""Build inspector cards from the firmware's declared values and wire schema."""
import hashlib
from functools import lru_cache
import re

from knowledge_subsystems import EXTRA_TOPICS, read_subsystems

SRC = "sketches/dual_track_control/src/"
WIKI = "docs/wiki/"
THERM = "config/ThermalConfig.h"
BATT = "config/BatteryConfig.h"
ALERT = "config/AlertConfig.h"
DRIVE = "config/DriveConfig.h"
GATE = "domain/safety/OutputGate.cpp"
SUPERVISOR = "domain/safety/SafetySupervisor.cpp"
BATTERY = "domain/battery/BatteryLadder.cpp"
THERMAL = "domain/thermal/ThermalDerating.cpp"
HYSTERESIS = "domain/thermal/ThermalHysteresis.cpp"
POLICY = "alerts/AlertPolicy.cpp"
PLAYER = "alerts/PatternPlayer.cpp"
SERVER = "infrastructure/network/DashboardServer.cpp"
WIFI = "infrastructure/network/WifiService.h"
BUS = "infrastructure/xc/XbusTelemetryAdapter.h"
JSON = "telemetry/JsonEncoder.cpp"

# Each card names its implementation. Numeric substitutions are read at build time.
CARDS = {
    "Safety": {
        "Drive permission": (
            "The first blocking reason wins: invalid RC, battery startup gate, battery cutoff, then thermal cutoff. "
            "The RC frame becomes stale after {SBUS_TIMEOUT} ms. The loop watchdog resets the MCU after {WDT_TIMEOUT_MS} ms without service.",
            SUPERVISOR, "application/FirmwareApp.cpp"),
        "What cutoff actually does": (
            "Both ESC commands ramp from their last written values to {SVC} µs over {CUTOFF_HOLD_MS} ms, "
            "then PWM detaches. Battery power remains connected. The ESCs can sound their own loss-of-signal beep.",
            GATE, "application/MotorOutput.cpp"),
        "Drive recovery": (
            "RC recovery and cooling can reopen the gate when other conditions allow. "
            "Battery Eco and battery cutoff latch until power-cycle. After boot, both packs must supply plausible "
            "readings with the lower pack at or above {CUTOFF_THRESH_V} V, or the boot gate times out after {BATTERY_CONFIRM_MS} ms.",
            SUPERVISOR, BATTERY),
    },
    "Thermal stages": {
        "Warning · sound only": (
            "Trips at ≥ {TEMP_WARN_ON_C} °C for {TEMP_DEBOUNCE_MS} ms. Clears below {TEMP_WARN_OFF_C} °C. Plays the warning trill when it has priority.",
            THERMAL, HYSTERESIS, POLICY),
        "Throttle · forced Eco": (
            "Trips at ≥ {TEMP_ECO_ON_C} °C for {TEMP_DEBOUNCE_MS} ms. Clears below {TEMP_ECO_OFF_C} °C. "
            "Forces Eco gear (scale {GEAR_LOW_SCALE} of average track command).",
            THERMAL, HYSTERESIS, "application/OperatorInput.cpp"),
        "Cutoff · stop propulsion": (
            "Trips at ≥ {TEMP_CUT_ON_C} °C for {TEMP_DEBOUNCE_MS} ms. Clears below {TEMP_CUT_OFF_C} °C. "
            "Commands the output gate to stop propulsion, plays the three-long-beep alarm, then queues the recovery flourish when the stage releases. "
            "The cutoff setting is provisional in firmware comments. The GL10’s internal thermal limit is unpublished.",
            THERMAL, HYSTERESIS, "application/SafetyControl.cpp", POLICY, GATE),
        "Sensor selection": (
            "Uses the hottest plausible smoothed ESC or motor temperature from either valid ESC. "
            "Accepts {TEMP_PLAUS_MIN_C} to {TEMP_PLAUS_MAX_C} °C inclusive. With no valid plausible reading, stages hold.",
            THERMAL, "application/SafetyControl.cpp"),
    },
    "Battery thresholds": {
        "Throttle · forced Eco": (
            "Lower pack voltage < {ECO_LOCK_THRESH_V} V for {ECO_LOCK_DEBOUNCE_MS} ms forces Eco until power-cycle.",
            BATTERY),
        "Warning · sound only": (
            "Lower pack voltage < {LOWV_THRESH_V} V for {LOWV_DEBOUNCE_MS} ms latches the low-battery chirps after a {ALERT_STARTUP_MS} ms startup grace.",
            POLICY),
        "Cutoff · stop propulsion": (
            "Lower pack voltage < {CUTOFF_THRESH_V} V for {CUTOFF_DEBOUNCE_MS} ms latches cutoff and requests low-battery chirps immediately. "
            "The motor protection has its own validity checks and runs during the audible alarm’s startup grace.",
            BATTERY, "application/SafetyControl.cpp", GATE),
        "Reading validity": (
            "Uses the lower smoothed voltage of two valid packs. The configured plausibility band is {LOWV_PLAUS_MIN_V}–{LOWV_PLAUS_MAX_V} V inclusive. "
            "Invalid readings reset the pending battery debounce. Already latched stages hold.",
            BATTERY, POLICY, "domain/battery/VoltagePlausibility.cpp"),
    },
    "Beeps": {
        "Priority and output": (
            "Repeating priority: thermal cutoff → low battery/battery cutoff → thermal warning → RC off. "
            "Manual horn, one-shot patterns, and the selected repeating alarm share D{PIN_BEEPER}; any active output drives the piezo. "
            "Horn: valid RC and its horn channel above {HORN_ON_RAW} raw.",
            POLICY, "application/AlertControl.cpp", "application/FirmwareApp.cpp"),
        "Thermal cutoff · three long": ("Repeats while the thermal cutoff alarm has priority. {ALERT_THERM_CUT}", POLICY),
        "Low battery · three short": ("Repeats until power-cycle when selected. {ALERT_LOWV}", POLICY),
        "Thermal warning · trill": ("Repeats while selected. {ALERT_THERM_WARN}", POLICY),
        "RC off · one long": ("Starts after {INACT_RC_OFF_MS} ms without valid RC. Clears when valid RC returns. {ALERT_INACT}", POLICY),
        "Thermal recovery · four quick": ("One-shot when thermal cutoff clears. {BEEP_THERM_RESTORED}", "application/SafetyControl.cpp", PLAYER),
        "Wi-Fi ready · two short": ("One-shot when Wi-Fi is ready. {BEEP_WIFI_READY}", "infrastructure/network/WifiService.cpp", PLAYER),
    },
    "Data and API": {
        "Dashboard snapshot and event stream": (
            "{routes}. The SSE stream sends the same JSON shape every {SSE_INTERVAL_MS} ms when servicing a client. "
            "These are read-only observation routes. The server falls back to the dashboard page for other requests.",
            SERVER, "infrastructure/network/SseStream.cpp"),
        "Frame metadata": (
            "t is the controller time in ms, seq is the sequence of built JSON frames, gear is the active gear, and mode is the operator authority mode.",
            JSON),
        "Safety fields": (
            "Raw RC receiver failsafe/lost-frame flags: fs, lost. Battery: eco, cut. Thermal: hot, teco, tcut. "
            "Last written servo pulses in µs: outL, outR. Flags encode zero or one.",
            JSON),
        "ESC measurements": (
            "e0 and e1 carry validity (ok), age in ms, mechanical rpm, current in tenths of an amp (cur), voltage in tenths "
            "of a volt (v), and whole °C temperatures (tE = ESC, tM = motor). X.BUS data becomes invalid after {TELEMETRY_STALE_MS} ms without a good reply.",
            JSON, "infrastructure/xc/XbusTelemetryAdapter.cpp"),
        "JSON keys in the firmware encoder": ("{fields}.", JSON),
    },
}

SAFETY, HEAT, VOLTAGE, SOUND, DATA = CARDS
TOPICS = {
    "safety-system": (SAFETY, HEAT, VOLTAGE, SOUND, DATA),
    "thermal-derating": (HEAT, SAFETY, SOUND, DATA),
    "battery-ladder": (VOLTAGE, SAFETY, SOUND, DATA),
    "output-gate": (SAFETY, HEAT, VOLTAGE, SOUND, DATA),
    "alerts-and-alarms": (SOUND, HEAT, VOLTAGE, SAFETY),
    "sbus-input": (SAFETY, SOUND),
    "gear-and-reverse-caps": (HEAT, VOLTAGE),
    "telemetry-and-dashboard": (DATA, HEAT, VOLTAGE),
    "wifi-dashboard": (DATA,),
    "xbus-telemetry": (DATA, HEAT, VOLTAGE),
}
CONFIGS = (THERM, BATT, ALERT, DRIVE, "config/SafetyConfig.h", "config/InputConfig.h",
           "config/Pins.h", "application/OperatorInput.h", WIFI, BUS)
DECLARATION = re.compile(
    r"^\s*(?:inline\s+)?const(?:expr)?\s+\w+\s+([A-Z0-9_]+)(?:\[\])?\s*=\s*([^;]+);", re.M
)
NUMBER = re.compile(r"(-?(?:\d+(?:\.\d+)?|\.\d+))[fFuUlL]*")


def read_cards(root, files):
    """Return display cards and their exact firmware references."""
    @lru_cache(maxsize=None)
    def read(path):
        path = SRC + path
        if path not in files:
            raise ValueError(f"Inspector source must be a tracked file: {path}")
        return (root / path).read_text(encoding="utf-8")

    values = {}
    locations = {}
    for path in CONFIGS:
        source = read(path)
        for match in DECLARATION.finditer(source):
            name, raw = match.groups()
            if name in values:
                raise ValueError(f"Duplicate inspector constant: {name}")
            values[name] = raw.strip()
            locations[name] = (path, source.count("\n", 0, match.start(1)) + 1)

    routes = re.findall(r'strstr\(req,\s*"(/[^"]+)"\)', read(SERVER))
    if set(routes) != {"/data", "/events"}:
        raise ValueError("Dashboard routes changed. Review the knowledge map API cards.")
    fields = list(dict.fromkeys(re.findall(r'\\"(\w+)\\":', read(JSON))))
    expected = set("t seq gear mode fs lost eco cut hot teco tcut outL outR e0 e1 ok age rpm cur v tE tM".split())
    if not expected <= set(fields):
        raise ValueError(f"Dashboard fields changed: {', '.join(sorted(expected - set(fields)))}. Review API cards.")

    covered = set()

    class CardValues(dict):
        def __missing__(self, key):
            if key not in values:
                raise ValueError(f"Missing inspector constant: {key}")
            covered.add(key)
            path, line = locations[key]
            sources.add((path, line))
            raw = values[key]
            if raw.startswith("{") and raw.endswith("}"):
                phases = [part.strip() for part in raw[1:-1].rstrip().rstrip(",").split(",")]
                if phases and all(phase.isdecimal() for phase in phases):
                    if values.get(key + "_LEN") != str(len(phases)):
                        raise ValueError(f"Pattern length does not match phases: {key}")
                    covered.add(key + "_LEN")
                    return "On/off phases (ms): " + " / ".join(phases) + "."
            if number := NUMBER.fullmatch(raw):
                if key == "SBUS_TIMEOUT":
                    return f"{float(number[1]) / 1000:g}"
                return number[1]
            raise ValueError(f"Unsupported inspector constant {key}: {raw}")

    result = {}
    for group, cards in CARDS.items():
        result[group] = []
        for label, (text, *implementations) in cards.items():
            sources = {(path, None) for path in implementations}
            description = text.format_map(CardValues(
                routes="GET /data (JSON) · GET /events (server-sent events)",
                fields=", ".join(fields)))
            for path, _ in sources:
                read(path)
            result[group].append({
                "label": label, "description": description,
                "sources": [{"path": SRC + path, **({"line": line} if line else {})}
                            for path, line in sorted(sources, key=lambda item: (item[0], item[1] or 0))],
            })
    unlisted = sorted(name for name, (path, _) in locations.items()
                      if path in {THERM, BATT, ALERT, "config/SafetyConfig.h"} and name not in covered)
    if unlisted:
        raise ValueError("Safety/alert settings lack inspector coverage: " + ", ".join(unlisted))
    return result, {path: hashlib.sha256(read(path).encode()).hexdigest()
                    for path in sorted({source["path"][len(SRC):] for rows in result.values()
                                        for card in rows for source in card["sources"]})}


# The explorer places a few primary files on the topic that explains them.
# Other uses stay references, and the file browser retains the actual path.
BRANCH_ARTIFACTS = {
    "output-gate": [(SRC + "domain/safety/OutputGate.cpp", "Code"), (SRC + "application/MotorOutput.cpp", "Code")],
    "drive-safeguards": [(SRC + "domain/safety/SafetySupervisor.cpp", "Code"),
                         (SRC + "application/FirmwareApp.cpp", "Code")],
    "battery-ladder": [(SRC + "domain/battery/BatteryLadder.cpp", "Code"),
                       (SRC + "config/BatteryConfig.h", "Configuration")],
    "thermal-derating": [(SRC + "domain/thermal/ThermalDerating.cpp", "Code"),
                         (SRC + "config/ThermalConfig.h", "Configuration")],
    "notification-channels": [(SRC + "application/AlertControl.cpp", "Code"),
                              (SRC + "alerts/AlertPolicy.cpp", "Code"),
                              (SRC + "alerts/PatternPlayer.cpp", "Code"),
                              (SRC + "config/AlertConfig.h", "Configuration")],
    "dashboard-display": [("dashboard/index.html", "Screen")],
    "wifi-dashboard": [(SRC + "infrastructure/network/DashboardServer.cpp", "Code"),
                       (SRC + "infrastructure/network/SseStream.cpp", "Code")],
    "xbus-telemetry": [(SRC + "infrastructure/xc/XbusTelemetryAdapter.cpp", "Code"),
                       ("docs/XBUS-PROTOCOL.md", "Reference")],
    "telemetry-interfaces": [(SRC + "telemetry/JsonEncoder.cpp", "Code"),
                             (SRC + "telemetry/CsvEncoder.cpp", "Code")],
    "sbus-input": [(SRC + "infrastructure/radiolink/SbusReceiverAdapter.cpp", "Code")],
    "gear-and-reverse-caps": [(SRC + "domain/drive/GearPolicy.cpp", "Code"),
                             (SRC + "config/DriveConfig.h", "Configuration")],
    "override-modes": [(SRC + "domain/drive/CommandMixer.cpp", "Code")],
    "testing": [("tests/invariants/test_invariant_stale_rc_neutral.cpp", "Test"),
                ("tests/telemetry/test_observer_encoders.cpp", "Test")],
}


# A short list of cross-branch links worth showing in the tree.
# Each endpoint must still contain its evidence before a link can be emitted.
BRANCH_CONNECTIONS = [
    (SRC + "application/FirmwareApp.cpp", SRC + "domain/safety/SafetySupervisor.cpp",
     "Calls the drive permission check", "safetySupervisorDecide(", "safetySupervisorDecide("),
    (SRC + "application/FirmwareApp.cpp", SRC + "application/MotorOutput.cpp",
     "Passes the drive decision and track commands", "outputUpdate(", "outputUpdate("),
    (SRC + "application/MotorOutput.cpp", SRC + "domain/safety/OutputGate.cpp",
     "Applies the output gate decision", "outputGateStep(", "outputGateStep("),
    (SRC + "application/AlertControl.cpp", SRC + "alerts/AlertPolicy.cpp",
     "Selects the active alarm pattern", "alertPatternSelect(", "alertPatternSelect("),
    (SRC + "application/AlertControl.cpp", SRC + "alerts/PatternPlayer.cpp",
     "Steps one-shot playback", "oneShotPatternStep(", "oneShotPatternStep("),
    (SRC + "infrastructure/network/DashboardServer.cpp", SRC + "infrastructure/network/SseStream.cpp",
     "Hands the event route to the stream", "sseAccept(", "sseAccept("),
    ("dashboard/index.html", SRC + "infrastructure/network/DashboardServer.cpp",
     "Subscribes to the /events route", "new EventSource(EVENTS_URL)", 'strstr(req, "/events")'),
    (SRC + "telemetry/JsonEncoder.cpp", "dashboard/index.html",
     "Provides the dashboard's ESC fields and safety flags", r'\"tcut\":', "if(d.tcut)"),
]


def enrich(graph, root, files):
    cards, firmware_inputs = read_cards(root, files)
    more_cards, more_inputs = read_subsystems(root, files)
    cards.update(more_cards)
    firmware_inputs.update(more_inputs)
    nodes = {node["id"]: node for node in graph["nodes"]}
    edges = graph["edges"]
    existing = {(edge["from"], edge["to"], edge["type"]) for edge in edges}

    def add_source(owner, path):
        if path not in nodes:
            nodes[path] = {"id": path, "title": path.rsplit("/", 1)[-1], "kind": "file", "summary": ""}
        key = owner, path, "source"
        if key not in existing:
            existing.add(key)
            edges.append({"from": owner, "to": path, "type": "source", "label": nodes[path]["title"]})

    for name in sorted(TOPICS.keys() | EXTRA_TOPICS.keys()):
        groups = TOPICS.get(name, ()) + EXTRA_TOPICS.get(name, ())
        path = WIKI + name + ".md"
        if path not in nodes:
            raise ValueError(f"Missing inspector topic: {path}")
        nodes[path]["sections"] = list(groups)
        for group in groups:
            for card in cards[group]:
                for source in card["sources"]:
                    add_source(path, source["path"])

    for topic, artifacts in BRANCH_ARTIFACTS.items():
        owner = WIKI + topic + ".md"
        if owner not in nodes:
            raise ValueError(f"Missing branch topic: {owner}")
        for path, role in artifacts:
            if path not in files or nodes[path]["kind"] != "file" or "branchParent" in nodes[path]:
                raise ValueError(f"Invalid or duplicate branch artifact: {path}")
            add_source(owner, path)
            nodes[path]["branchParent"] = owner
            nodes[path]["branchRole"] = role
            if role in {"Code", "Configuration", "Test"}:
                nodes[path]["branchLabel"] = path.rsplit("/", 1)[-1]
            elif role == "Screen":
                nodes[path]["branchLabel"] = "Live dashboard screen"
                nodes[path]["summary"] = ("Read-only dashboard showing per-track readings, drive settings, "
                                          "safety banners, and connection status.")

    for caller, target, label, left, right in BRANCH_CONNECTIONS:
        for path, token in ((caller, left), (target, right)):
            if not nodes[path].get("branchParent") or token not in (root / path).read_text(encoding="utf-8"):
                raise ValueError(f"Review branch relationship '{label}': evidence changed in {path}")
        edges.append({"from": caller, "to": target, "type": "connection", "label": label})

    governance = WIKI + "agent-governance.md"
    for path in sorted(files):
        if (path.startswith((".claude/agents/", ".claude/rules/")) and path.endswith(".md")
                or path.startswith(".claude/skills/") and path.endswith("/SKILL.md")):
            add_source(governance, path)

    # Source nodes show the cards they directly substantiate, without duplicating the payload.
    for group, rows in cards.items():
        for index, card in enumerate(rows):
            for path in {source["path"] for source in card["sources"]}:
                nodes[path].setdefault("cardRefs", {}).setdefault(group, []).append(index)
                if not nodes[path]["summary"]:
                    nodes[path]["summary"] = card["label"] + ": " + card["description"]
    graph["edges"] = sorted(edges, key=lambda e: (e["from"], e["to"], e["type"], e.get("anchor", "")))
    graph["cards"] = cards
    graph["firmwareInputs"] = firmware_inputs
    graph["nodes"] = sorted(nodes.values(), key=lambda node: node["id"])
    return graph
