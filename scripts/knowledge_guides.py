"""Curated paths explain the direction and purpose of repository relationships."""
from pathlib import PurePosixPath

ROOT = "sketches/dual_track_control/src/"
GOV = "docs/wiki/agent-governance.md"
XB = ROOT + "infrastructure/xc/XbusTelemetryAdapter.cpp"
CACHE = ROOT + "application/AlertControl.cpp"
MON = ROOT + "application/Monitoring.cpp"
JSON = ROOT + "telemetry/JsonEncoder.cpp"
CSV = ROOT + "telemetry/CsvEncoder.cpp"
SSE = ROOT + "infrastructure/network/SseStream.cpp"
HTTP = ROOT + "infrastructure/network/DashboardServer.cpp"
UI = "dashboard/index.html"
SAFETY = ROOT + "application/SafetyControl.cpp"

def add_guided_views(graph, root, nodes, edges):
    for path, title, description in (
        (".claude/skills", "Agent skills", "Runbooks for recurring tasks: safety review, field tuning, flashing, wiki review, and pull requests."),
        (".claude/agents", "Reviewer agents", "Specialist reviewers for architecture, safety, tests, and documentation."),
        (".claude/rules", "Project rules", "Constraints the agents follow, including behavior preservation and firmware timing."),
        (".claude/hooks", "Agent hooks", "Checks run during agent work."),
        (".githooks", "Git hooks", "Local Git checks when the hook path is configured."),
        (".github/workflows", "GitHub workflows", "Repository checks, review and board automation, and scheduled documentation work."),
    ):
        nodes[path].update(branchParent=GOV, branchLabel=title, summary=description)
        for child in nodes.values():
            if child["id"].startswith(path + "/"):
                child["ownerDomain"] = GOV
                if child["id"].endswith("/SKILL.md"):
                    child["branchLabel"] = PurePosixPath(child["id"]).parent.name
                    child["branchRole"] = "Skill"
    for path, title in (("CLAUDE.md", "Project instructions"), (".claude/settings.json", "Agent settings"),
                        (".github/copilot-instructions.md", "Copilot instructions"),
                        (".coderabbit.yaml", "CodeRabbit configuration")):
        nodes[path].update(branchParent=GOV, branchLabel=title, branchRole="Configuration")

    guide = [
        ("Live dashboard stream", "The browser subscribes to /events. The server builds telemetry from the current firmware snapshot and sends it to the browser.", [
            (XB, "Polls both ESCs over X.BUS and decodes replies.", "telemetrySourceUpdate"),
            (CACHE, "Maintains the shared ESC readings.", "telemetrySourceUpdate(telem"),
            (MON, "Copies the readings and controller state into a snapshot.", "snapshot.esc[0] = telem[0]"),
            (JSON, "Encodes the snapshot as dashboard JSON.", "encodeTelemetryJson"),
            (SSE, "Streams JSON as server-sent events.", "telemetryFrameBuild("),
            (UI, "Receives events and updates gauges and banners.", "new EventSource(EVENTS_URL)"),
        ]),
        ("On-demand JSON snapshot", "The /data endpoint returns a single JSON sample. The live dashboard receives /events.", [
            (MON, "Builds the current snapshot.", "buildSystemSnapshot"),
            (JSON, "Encodes the snapshot as JSON.", "encodeTelemetryJson"),
            (HTTP, "Returns JSON from the /data route.", "sendDataResponse(client)"),
        ]),
        ("Local protection", "Firmware makes protection decisions locally. The dashboard displays the resulting state.", [
            (XB, "Decodes voltage and temperature from the ESCs.", "telemetrySourceUpdate"),
            (CACHE, "Holds shared readings for the firmware.", "telemetrySourceUpdate(telem"),
            (SAFETY, "Uses voltage and temperature for battery and thermal protection.", "telem[i].motorTempC"),
        ]),
        ("USB serial diagnostics", "The debug console emits CSV from the firmware snapshot.", [
            (MON, "Samples the same firmware state for debug output.", "encodeDebugCsv(buf"),
            (CSV, "Formats the snapshot as a CSV record.", "encodeDebugCsv"),
        ]),
        ("Wiki maintenance and CI", "The skill guides contributors. The workflow independently runs the verification scripts.", [
            (".claude/skills/wiki-impact-review/SKILL.md", "Review affected notes and refresh the graph.", "build_knowledge_graph.py"),
            (".github/workflows/wiki-lint.yml", "Runs wiki and graph verification in GitHub Actions.", "build_knowledge_graph.py --check"),
            ("scripts/build_knowledge_graph.py", "Verifies the generated HTML against the tracked sources.", 'parser.add_argument("--check"'),
        ]),
    ]
    graph["guidedViews"] = []
    for title, summary, steps in guide:
        path_steps = []
        for path, label, token in steps:
            source = (root / path).read_text(encoding="utf-8")
            line = next((i for i, row in enumerate(source.splitlines(), 1) if token in row), None)
            if not line or path not in nodes:
                raise ValueError(f"Missing evidence for {title}: {path}: {token}")
            path_steps.append({"node": path, "label": label, "line": line})
        graph["guidedViews"].append({"title": title, "summary": summary, "steps": path_steps,
                                      "area": "governance" if title.startswith("Wiki") else "telemetry"})
