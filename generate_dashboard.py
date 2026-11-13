import csv
import json
import html
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = BASE_DIR / "outputs"
CSV_PATH = OUTPUT_DIR / "all_targets_benchmark.csv"
METRICS_PATH = OUTPUT_DIR / "metrics.json"
DASHBOARD_PATH = OUTPUT_DIR / "dashboard.html"
MAP_IMAGE = "final_run.png"


def load_results():
    missing = [p.name for p in (CSV_PATH, METRICS_PATH) if not p.exists()]
    if missing:
        raise FileNotFoundError(
            "Missing required file(s) in outputs/: " + ", ".join(missing)
            + "\\nRun main.py first to generate the simulation results."
        )

    with CSV_PATH.open("r", newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    with METRICS_PATH.open("r", encoding="utf-8") as f:
        metrics = json.load(f)

    required = {
        "target_number", "target_row", "target_column", "algorithm",
        "reachable", "path_length", "explored_cells", "runtime_ms"
    }
    if not rows:
        raise ValueError(f"{CSV_PATH.name} contains no benchmark rows.")
    if not required.issubset(rows[0].keys()):
        raise ValueError("Unexpected CSV columns. Required: " + ", ".join(sorted(required)))

    for row in rows:
        row["target_number"] = int(row["target_number"])
        row["target_row"] = int(row["target_row"])
        row["target_column"] = int(row["target_column"])
        row["algorithm"] = row["algorithm"].strip().upper()
        row["reachable"] = str(row["reachable"]).strip().lower() in ("true", "1", "yes")
        row["path_length"] = int(float(row["path_length"]))
        row["explored_cells"] = int(float(row["explored_cells"]))
        row["runtime_ms"] = float(row["runtime_ms"])

    rows.sort(key=lambda r: (r["target_number"], r["algorithm"]))
    return rows, metrics


def build_dashboard(rows, metrics):
    targets = sorted({r["target_number"] for r in rows})
    by_target = {}
    for number in targets:
        matching = [r for r in rows if r["target_number"] == number]
        by_target[number] = {r["algorithm"]: r for r in matching}

    def target_sample(n):
        return by_target[n].get("BFS", by_target[n].get("ASTAR", {}))

    labels = [
        f'T{n} ({target_sample(n).get("target_row", "?")},{target_sample(n).get("target_column", "?")})'
        for n in targets
    ]

    def values(algorithm, field):
        return [by_target[n].get(algorithm, {}).get(field) for n in targets]

    bfs_explored, astar_explored = values("BFS", "explored_cells"), values("ASTAR", "explored_cells")
    bfs_path, astar_path = values("BFS", "path_length"), values("ASTAR", "path_length")
    bfs_runtime, astar_runtime = values("BFS", "runtime_ms"), values("ASTAR", "runtime_ms")

    bfs_total = sum(v for v in bfs_explored if v is not None)
    astar_total = sum(v for v in astar_explored if v is not None)
    reduction = ((bfs_total - astar_total) / bfs_total * 100) if bfs_total else None

    def js(value):
        return json.dumps(value, ensure_ascii=False)

    coverage = float(metrics.get("target_coverage_percent", 0))
    visited = int(metrics.get("targets_visited", 0))
    total_targets = int(metrics.get("total_targets", len(targets)))
    steps = int(metrics.get("path_length_grid_steps", 0))
    obstacles = int(metrics.get("dynamic_obstacles_discovered", 0))
    replans = int(metrics.get("replanning_count", 0))
    mission_time = float(metrics.get("mission_runtime_seconds", 0))
    planning_ms = float(metrics.get("robot_total_planning_time_ms", 0))
    explored_sum = int(metrics.get("robot_total_explored_cells", metrics.get("explored_cells_sum_per_plan", 0)))
    grid_size = metrics.get("grid_size", [])
    grid_label = " × ".join(str(x) for x in grid_size) if grid_size else "N/A"
    algorithm = str(metrics.get("selected_algorithm", "N/A")).upper()

    table_rows = []
    for n in targets:
        pair = by_target[n]
        sample = target_sample(n)
        cells = []
        for alg in ("BFS", "ASTAR"):
            r = pair.get(alg)
            if r:
                status = "Reachable" if r["reachable"] else "Unreachable"
                cells.extend([
                    f"<td>{status}</td>",
                    f"<td>{r['path_length'] if r['reachable'] else '—'}</td>",
                    f"<td>{r['explored_cells']:,}</td>",
                    f"<td>{r['runtime_ms']:.4f}</td>"
                ])
            else:
                cells.extend(["<td>Missing</td>"] * 4)
        table_rows.append(
            f"<tr><td>{n}</td><td>({sample.get('target_row', '?')}, {sample.get('target_column', '?')})</td>"
            + "".join(cells) + "</tr>"
        )
    table_html = "\\n".join(table_rows)
    note = html.escape(str(metrics.get("notes", "")))
    matching_paths = sum(
        1 for n in targets
        if by_target[n].get("BFS") and by_target[n].get("ASTAR")
        and by_target[n]["BFS"]["reachable"] and by_target[n]["ASTAR"]["reachable"]
        and by_target[n]["BFS"]["path_length"] == by_target[n]["ASTAR"]["path_length"]
    )

    return f'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Autonomous Search &amp; Rescue Robot Dashboard</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<style>
:root{{--bg:#0b1120;--panel:#121c2f;--panel2:#17243a;--text:#edf3ff;--muted:#9eacc4;--line:#2b3a54;--accent:#61d6b3;--blue:#77a9ff;}}
*{{box-sizing:border-box}}
body{{margin:0;background:radial-gradient(ellipse at top,#172641 0,#0b1120 58%);color:var(--text);font-family:Inter,Segoe UI,Arial,sans-serif}}
main{{max-width:1440px;margin:auto;padding:30px 22px 46px}}
header{{display:flex;justify-content:space-between;align-items:flex-start;gap:16px;flex-wrap:wrap;margin-bottom:24px}}
.eyebrow{{color:var(--accent);text-transform:uppercase;letter-spacing:2px;font-size:12px;font-weight:800}}
h1{{font-size:clamp(26px,4vw,42px);line-height:1.12;margin:9px 0}}
.subtitle{{color:var(--muted);margin:0;line-height:1.6}}
.pill{{border:1px solid #316653;color:var(--accent);border-radius:999px;padding:9px 13px;font-size:12px;font-weight:700;white-space:nowrap}}
.grid{{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:15px;margin-bottom:18px}}
.card,.panel{{background:linear-gradient(145deg,var(--panel),#10192a);border:1px solid var(--line);border-radius:17px;box-shadow:0 10px 28px #0002}}
.card{{padding:19px;min-height:120px}}
.kpi-label{{color:var(--muted);font-size:12px;text-transform:uppercase;letter-spacing:1px;font-weight:700}}
.kpi{{font-size:clamp(26px,3vw,36px);font-weight:800;margin:13px 0 5px;letter-spacing:-1px}}
.kpi-sub{{color:var(--muted);font-size:12px}}
.two{{display:grid;grid-template-columns:1fr 1fr;gap:18px;margin-bottom:18px}}
.panel{{padding:20px;min-width:0;margin-bottom:18px}}
.two .panel{{margin-bottom:0}}
h2{{font-size:17px;margin:0 0 5px}}
.desc{{color:var(--muted);font-size:12px;line-height:1.5;margin:0 0 18px}}
.chartbox{{height:280px;position:relative}}
.map{{width:100%;display:block;border-radius:12px;border:1px solid var(--line);background:#09101d;max-height:640px;object-fit:contain}}
.map-placeholder{{padding:25px;background:var(--panel2);border-radius:12px;color:var(--muted)}}
.table-wrap{{overflow-x:auto}}
table{{width:100%;border-collapse:collapse;min-width:850px;font-size:13px}}
th{{text-align:right;color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.6px;background:#1a2940}}
th:first-child,th:nth-child(2),td:first-child,td:nth-child(2){{text-align:left}}
th,td{{padding:13px 12px;border-bottom:1px solid var(--line);white-space:nowrap}}
thead tr:first-child th{{border-bottom:1px solid #41516b}}
thead tr:nth-child(2) th{{font-size:10px}}
tbody tr:hover{{background:#ffffff08}}
.group-bfs{{color:var(--blue)}} .group-astar{{color:var(--accent)}}
.insights{{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:13px}}
.insight{{padding:15px;border-radius:12px;background:var(--panel2);border:1px solid var(--line);line-height:1.5;font-size:13px}}
.insight strong{{display:block;color:var(--accent);margin-bottom:6px;font-size:14px}}
footer{{color:var(--muted);font-size:11px;line-height:1.7;margin-top:18px}}
@media(max-width:900px){{.grid{{grid-template-columns:repeat(2,minmax(0,1fr))}}.two{{grid-template-columns:1fr}}.insights{{grid-template-columns:1fr}}}}
@media(max-width:480px){{main{{padding:20px 12px}}.grid{{gap:9px}}.card{{padding:14px;min-height:110px}}.kpi{{font-size:26px}}}}
</style>
</head>
<body><main>
<header><div><div class="eyebrow">Robotics Vision · Simulation Analytics</div>
<h1>Autonomous Search &amp; Rescue Robot</h1>
<p class="subtitle">Mission performance, path-planning benchmarks, and obstacle discovery.</p></div>
<div class="pill">LATEST SAVED RESULTS · {html.escape(algorithm)}</div></header>

<section class="grid">
<article class="card"><div class="kpi-label">Target coverage</div><div class="kpi">{coverage:.1f}%</div><div class="kpi-sub">{visited} of {total_targets} targets visited</div></article>
<article class="card"><div class="kpi-label">Mission path length</div><div class="kpi">{steps:,} <span style="font-size:14px;color:var(--muted)">steps</span></div><div class="kpi-sub">Grid movement steps</div></article>
<article class="card"><div class="kpi-label">Obstacles discovered</div><div class="kpi">{obstacles}</div><div class="kpi-sub">Dynamic obstacles detected</div></article>
<article class="card"><div class="kpi-label">Replanning events</div><div class="kpi">{replans}</div><div class="kpi-sub">Mission recovery and rerouting</div></article>
<article class="card"><div class="kpi-label">Mission runtime</div><div class="kpi">{mission_time:.3f} <span style="font-size:14px;color:var(--muted)">sec</span></div><div class="kpi-sub">Recorded mission runtime</div></article>
<article class="card"><div class="kpi-label">Planning time</div><div class="kpi">{planning_ms:.2f} <span style="font-size:14px;color:var(--muted)">ms</span></div><div class="kpi-sub">Accumulated planning time</div></article>
<article class="card"><div class="kpi-label">Explored-cell total</div><div class="kpi">{explored_sum:,}</div><div class="kpi-sub">May count cells across repeated plans</div></article>
<article class="card"><div class="kpi-label">Environment</div><div class="kpi">{html.escape(grid_label)}</div><div class="kpi-sub">Rows × columns</div></article>
</section>

<section class="two">
<article class="panel"><h2>Explored cells by target</h2><p class="desc">Lower counts generally mean less search work for a target.</p><div class="chartbox"><canvas id="exploredChart"></canvas></div></article>
<article class="panel"><h2>Path length by target</h2><p class="desc">Compare the number of grid steps returned by each algorithm.</p><div class="chartbox"><canvas id="pathChart"></canvas></div></article>
</section>
<section class="two">
<article class="panel"><h2>Planning runtime by target</h2><p class="desc">Runtime is measured in milliseconds and varies with system load.</p><div class="chartbox"><canvas id="runtimeChart"></canvas></div></article>
<article class="panel"><h2>Latest mission map</h2><p class="desc">Map image saved by the simulation run.</p>
<img class="map" src="{MAP_IMAGE}" alt="Final search and rescue robot mission map"
onerror="this.outerHTML='<div class=&quot;map-placeholder&quot;>Could not load final_run.png. Place dashboard.html beside final_run.png in outputs.</div>'"></article>
</section>

<section class="panel"><h2>BFS vs A* benchmark details</h2><p class="desc">Values are read from all_targets_benchmark.csv each time this dashboard is generated.</p>
<div class="table-wrap"><table>
<thead><tr><th rowspan="2">Target</th><th rowspan="2">Grid cell</th><th colspan="4" class="group-bfs">BFS</th><th colspan="4" class="group-astar">A*</th></tr>
<tr><th>Status</th><th>Path steps</th><th>Explored</th><th>Runtime (ms)</th><th>Status</th><th>Path steps</th><th>Explored</th><th>Runtime (ms)</th></tr></thead>
<tbody>{table_html}</tbody></table></div></section>

<section class="panel"><h2>Quick interpretation</h2><p class="desc">Automatically calculated from the benchmark CSV.</p>
<div class="insights">
<div class="insight"><strong>{f"{reduction:.1f}% fewer explored cells" if reduction is not None else "Explored-cell comparison unavailable"}</strong>
A* explored {astar_total:,} cells across benchmark targets compared with {bfs_total:,} for BFS. These are summed per-target searches.</div>
<div class="insight"><strong>Path lengths compared</strong>
{matching_paths} target(s) have matching reachable BFS and A* path lengths.</div>
<div class="insight"><strong>Interpret runtime carefully</strong>
A* may explore fewer cells, but individual runtime can vary with system load and small grid sizes.</div>
</div></section>
<footer>Generated from <code>outputs/all_targets_benchmark.csv</code> and <code>outputs/metrics.json</code>. Mission metrics describe the selected-algorithm run; benchmark results compare BFS and A*.<br>{note}</footer>
</main>
<script>
const labels = {js(labels)};
const bfsExplored = {js(bfs_explored)}, astarExplored = {js(astar_explored)};
const bfsPath = {js(bfs_path)}, astarPath = {js(astar_path)};
const bfsRuntime = {js(bfs_runtime)}, astarRuntime = {js(astar_runtime)};
Chart.defaults.color = '#b8c5da';
Chart.defaults.borderColor = '#2b3a54';
const common = {{responsive:true,maintainAspectRatio:false,interaction:{{mode:'index',intersect:false}},plugins:{{legend:{{labels:{{usePointStyle:true,boxWidth:8}}}}}},scales:{{x:{{grid:{{display:false}}}},y:{{beginAtZero:true,grid:{{color:'#26354b'}}}}}}}};
function makeChart(id, bfs, astar, yTitle) {{
 new Chart(document.getElementById(id), {{type:'bar',data:{{labels,datasets:[
 {{label:'BFS',data:bfs,backgroundColor:'#77a9ff',borderRadius:5}},
 {{label:'A*',data:astar,backgroundColor:'#61d6b3',borderRadius:5}}
 ]}},options:{{...common,scales:{{...common.scales,y:{{...common.scales.y,title:{{display:true,text:yTitle}}}}}}}}}});
}}
makeChart('exploredChart',bfsExplored,astarExplored,'Cells explored');
makeChart('pathChart',bfsPath,astarPath,'Grid steps');
makeChart('runtimeChart',bfsRuntime,astarRuntime,'Milliseconds');
</script></body></html>'''


def main():
    rows, metrics = load_results()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    DASHBOARD_PATH.write_text(build_dashboard(rows, metrics), encoding="utf-8")
    print("Dashboard generated successfully.")
    print(f"Benchmark rows loaded: {len(rows)}")
    print(f"Targets in benchmark: {len({r['target_number'] for r in rows})}")
    print(f"Coverage: {metrics.get('target_coverage_percent', 'N/A')}%")
    print(f"Dashboard: {DASHBOARD_PATH}")
    print("Open outputs/dashboard.html in your browser.")


if __name__ == "__main__":
    main()
