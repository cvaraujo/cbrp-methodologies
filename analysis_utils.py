"""
Shared utilities for parsing result files and exporting to LaTeX.
Used by analysis-stochastic-model.ipynb and analysis-simulated-annealing.ipynb.
"""
from __future__ import annotations

import os
import re
from pathlib import Path
from typing import Any


def _count_attended_blocks(y_str: str) -> int:
    """Count attended blocks from 'Y: 0,1,3,6,...' (empty = 0)."""
    if not y_str or not y_str.strip():
        return 0
    return len([x for x in y_str.split(",") if x.strip()])


def instance_sort_key(name: str) -> tuple[str, int, int]:
    """
    Return sort key for instance name: (city, map_size, numeric_id).
    Order: city (alto-santo, limoeiro), then map size (500, 1000, 2000), then id (1, 2, 3, ...).
    Examples: 'alto-santo-500-1' -> ('alto-santo', 500, 1); 'limoeiro-1000-2' -> ('limoeiro', 1000, 2).
    """
    parts = str(name).strip().split("-")
    if len(parts) < 2:
        return (str(name), 0, 0)
    # Last two numeric parts are size and id
    size, id_ = 0, 0
    if len(parts) >= 2 and parts[-1].isdigit():
        id_ = int(parts[-1])
    if len(parts) >= 3 and parts[-2].isdigit():
        size = int(parts[-2])
    city = "-".join(parts[:-2]) if len(parts) >= 3 else (parts[0] if parts else "")
    return (city, size, id_)


def parse_model_result(filepath: str | Path) -> dict[str, Any] | None:
    """
    Parse a stochastic model result file (e.g. cbrp-stoc output).
    Returns dict with: N, M, B, S, Alpha, LB, UB, Gurobi_Nodes, Lazy_cuts, Frac_cuts, Runtime,
    instance_name, attended_s0, attended_per_scenario, gap_pct (computed).
    """
    path = Path(filepath)
    if not path.is_file():
        return None
    text = path.read_text()
    lines = text.splitlines()
    data: dict[str, Any] = {
        "instance_name": path.stem,
        "attended_s0": 0,
        "attended_per_scenario": [],
        "attended_avg": 0.0,
    }
    i = 0
    while i < len(lines):
        line = lines[i]
        parts = line.split(maxsplit=1)
        if len(parts) < 2:
            i += 1
            continue
        key, rest = parts[0].rstrip(":"), parts[1].strip()
        if key == "N":
            data["N"] = int(rest)
        elif key == "M":
            data["M"] = int(rest)
        elif key == "B":
            data["B"] = int(rest)
        elif key == "S":
            data["S"] = int(rest)
        elif key == "Alpha":
            data["Alpha"] = float(rest)
        elif key == "LB":
            data["LB"] = float(rest)
        elif key == "UB":
            data["UB"] = float(rest)
        elif key == "Gurobi_Nodes":
            data["Gurobi_Nodes"] = int(rest)
        elif key == "Lazy_cuts":
            data["Lazy_cuts"] = int(rest)
        elif key == "Frac_cuts":
            data["Frac_cuts"] = int(rest)
        elif key == "Runtime":
            data["Runtime"] = float(rest)
        elif key == "Scenario":
            # "Scenario 0: " -> scenario index, next lines are X: and Y:
            scenario_idx = int(rest.split(":")[0].strip())
            i += 1
            attended = 0
            while i < len(lines):
                ln = lines[i]
                if ln.strip().startswith("Y:"):
                    y_part = ln.split("Y:", 1)[1].strip() if "Y:" in ln else ""
                    attended = _count_attended_blocks(y_part)
                    if scenario_idx == 0:
                        data["attended_s0"] = attended
                    data["attended_per_scenario"].append(attended)
                    i += 1
                    break
                if re.match(r"Scenario\s+\d+", ln.strip()):
                    break
                i += 1
            continue
        i += 1

    if "UB" in data and data["UB"] and data["UB"] > 0 and "LB" in data:
        data["gap_pct"] = 100.0 * (data["UB"] - data["LB"]) / data["UB"]
    else:
        data["gap_pct"] = None
    if data["attended_per_scenario"]:
        data["attended_avg"] = sum(data["attended_per_scenario"]) / len(data["attended_per_scenario"])
    return data


def parse_sa_result(filepath: str | Path) -> dict[str, Any] | None:
    """
    Parse a simulated annealing result file (cbrp-stoc-sa output).
    Returns dict with: N, M, B, S, Alpha, Start_LB, LB, Runtime, instance_name,
    attended_s0, attended_per_scenario, route_time_s0, attend_time_s0, improvement (LB - Start_LB).
    """
    path = Path(filepath)
    if not path.is_file():
        return None
    text = path.read_text()
    lines = text.splitlines()
    data: dict[str, Any] = {
        "instance_name": path.stem,
        "attended_s0": 0,
        "attended_per_scenario": [],
        "route_time_s0": None,
        "attend_time_s0": None,
    }
    i = 0
    while i < len(lines):
        line = lines[i]
        parts = line.split(maxsplit=1)
        if len(parts) < 2:
            i += 1
            continue
        key, rest = parts[0].rstrip(":"), parts[1].strip()
        if key == "N":
            data["N"] = int(rest)
        elif key == "M":
            data["M"] = int(rest)
        elif key == "B":
            data["B"] = int(rest)
        elif key == "S":
            data["S"] = int(rest)
        elif key == "Alpha":
            data["Alpha"] = float(rest)
        elif key == "Start_UB":
            data["Start_LB"] = float(rest)
        elif key == "LB":
            data["LB"] = float(rest)
        elif key == "Runtime":
            data["Runtime"] = float(rest)
        elif key == "Scenario":
            scenario_idx = int(rest.split(":")[0].strip())
            i += 1
            attended = 0
            while i < len(lines):
                ln = lines[i]
                if ln.strip().startswith("Y:"):
                    y_part = ln.split("Y:", 1)[1].strip() if "Y:" in ln else ""
                    attended = _count_attended_blocks(y_part)
                    if scenario_idx == 0:
                        data["attended_s0"] = attended
                    data["attended_per_scenario"].append(attended)
                    i += 1
                    break
                i += 1
            # Look for Route_Time and Attend_Time for this scenario
            while i < len(lines):
                ln = lines[i]
                if ln.strip().startswith("Route_Time:"):
                    if scenario_idx == 0:
                        data["route_time_s0"] = float(ln.split(":", 1)[1].strip())
                elif ln.strip().startswith("Attend_Time:"):
                    if scenario_idx == 0:
                        data["attend_time_s0"] = float(ln.split(":", 1)[1].strip())
                elif re.match(r"Scenario\s+\d+", ln.strip()):
                    break
                i += 1
            continue
        i += 1

    if "Start_LB" in data and "LB" in data:
        data["improvement"] = data["LB"] - data["Start_LB"]
    else:
        data["improvement"] = None
    return data


def parse_experiment_folder_model(
    base_dir: str | Path,
    pattern: str = "experiment-{alpha}-{model}-{model_type}-{use_preprocessing}",
) -> list[tuple[str, dict[str, Any]]]:
    """
    Scan stochastic-results-model folder for experiment dirs and parse all result .txt files.
    Returns list of (experiment_dir_name, parsed_result_dict) per file.
    """
    base = Path(base_dir)
    if not base.is_dir():
        return []
    out: list[tuple[str, dict[str, Any]]] = []
    for exp_dir in sorted(base.iterdir()):
        if not exp_dir.is_dir() or not exp_dir.name.startswith("experiment-"):
            continue
        for f in exp_dir.glob("*.txt"):
            if f.name.startswith("scenarios"):
                continue
            parsed = parse_model_result(f)
            if parsed:
                parsed["experiment"] = exp_dir.name
                out.append((exp_dir.name, parsed))
    return out


def parse_experiment_folder_sa(
    base_dir: str | Path,
) -> list[tuple[str, dict[str, Any]]]:
    """
    Scan stochastic-results-sa for experiment-* dirs and parse all result .txt files.
    Returns list of (experiment_dir_name, parsed_result_dict) per file.
    """
    base = Path(base_dir)
    if not base.is_dir():
        return []
    out: list[tuple[str, dict[str, Any]]] = []
    for exp_dir in sorted(base.iterdir()):
        if not exp_dir.is_dir() or not exp_dir.name.startswith("experiment-"):
            continue
        for f in exp_dir.glob("*.txt"):
            if f.name.startswith("scenarios"):
                continue
            parsed = parse_sa_result(f)
            if parsed:
                parsed["experiment"] = exp_dir.name
                out.append((exp_dir.name, parsed))
    return out


def _fmt_num(x: float | int | None, decimals: int = 2, use_comma: bool = True) -> str:
    if x is None:
        return "-"
    if isinstance(x, int):
        return str(x)
    s = f"{x:.{decimals}f}"
    if use_comma:
        s = s.replace(".", ",")
    return s


def export_model_config_to_latex(
    df,
    caption: str,
    label: str,
    use_rowcolor: bool = True,
) -> str:
    """
    Export a DataFrame of model results to thesis-style LaTeX table.
    df must have columns: Instance (or instance_name), |V| (or N), |A| (or M), |B| (or B),
    Attended Blocks (or attended_s0), LB, UB, gap (%), Gurobi_Nodes, Lazy_cuts, Runtime.
    Optional: Alpha, S for stochastic.
    """
    # Normalize column names
    col_map = {
        "instance_name": "Instance",
        "N": "|V|",
        "M": "|A|",
        "B": "|B|",
        "attended_s0": "Attended Blocks",
        "gap_pct": "gap (%)",
        "Gurobi_Nodes": "#B&B Nodes",
        "Lazy_cuts": "Lazy Cuts",
        "Runtime": "Exec. Time (s)",
    }
    rows: list[str] = []
    rows.append(r"\begin{table}[ht!]")
    rows.append(r"\centering")
    rows.append(rf"\caption{{{caption}}}")
    rows.append(rf"\label{{{label}}}")
    rows.append(r"\resizebox{\textwidth}{!}{%")
    rows.append(r"\begin{tabular}{lrrrrrrrrrrrr}")
    rows.append(r"\toprule")
    header = (
        r"\multicolumn{1}{c}{\textbf{Instance}} & "
        r"\multicolumn{1}{c}{\textbf{|V|}} & "
        r"\multicolumn{1}{c}{\textbf{|A|}} & "
        r"\multicolumn{1}{c}{\textbf{|B|}} & "
        r"\multicolumn{1}{c}{\textbf{S}} & "
        r"\multicolumn{1}{c}{\textbf{Alpha}} & "
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Attended\\ Blocks\end{tabular}}} & "
        r"\multicolumn{1}{c}{\textbf{LB}} & "
        r"\multicolumn{1}{c}{\textbf{UB}} & "
        r"\multicolumn{1}{c}{\textbf{gap (\%)}} & "
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}\#B\&B\\ Nodes\end{tabular}}} & "
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Lazy\\ Cuts\end{tabular}}} & "
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Exec.\\ Time (s)\end{tabular}}} \\"
    )
    rows.append(header)
    rows.append(r"\midrule")

    for idx, row in df.iterrows():
        inst = row.get("Instance", row.get("instance_name", ""))
        n = row.get("|V|", row.get("N", "-"))
        m = row.get("|A|", row.get("M", "-"))
        b = row.get("|B|", row.get("B", "-"))
        s = row.get("S", "-")
        alpha = row.get("Alpha", "-")
        if isinstance(alpha, float):
            alpha = _fmt_num(alpha, 1)
        att = row.get("Attended Blocks", row.get("attended_s0", "-"))
        lb = row.get("LB", "-")
        ub = row.get("UB", "-")
        gap = row.get("gap (%)", row.get("gap_pct"))
        if gap is not None:
            gap = _fmt_num(gap, 2)
        else:
            gap = "-"
        nodes = row.get("#B&B Nodes", row.get("Gurobi_Nodes", "-"))
        lazy = row.get("Lazy Cuts", row.get("Lazy_cuts", "-"))
        time_ = row.get("Exec. Time (s)", row.get("Runtime"))
        if time_ is not None:
            time_ = _fmt_num(time_, 2)
        else:
            time_ = "-"
        if isinstance(lb, (int, float)):
            lb = _fmt_num(lb, 0)
        if isinstance(ub, (int, float)):
            ub = _fmt_num(ub, 0)
        rowcolor = r"\rowcolor{rowgrey}" if use_rowcolor and (idx % 2 == 0) else (r"\rowcolor{rowlight}" if use_rowcolor else "")
        line = f"{rowcolor}{inst} & {n} & {m} & {b} & {s} & {alpha} & {att} & {lb} & {ub} & {gap} & {nodes} & {lazy} & {time_} \\\\"
        rows.append(line)
    rows.append(r"\bottomrule")
    rows.append(r"\end{tabular}%")
    rows.append(r"}")
    rows.append(r"\end{table}")
    return "\n".join(rows)


def export_sa_best_to_latex(
    df,
    caption: str,
    label: str,
    use_rowcolor: bool = True,
) -> str:
    """
    Export SA results (best configuration) to LaTeX.
    df columns: Instance, |V|, |A|, |B|, Alpha, Start_LB, LB, Improvement, Attended (s0), Runtime.
    """
    rows: list[str] = []
    rows.append(r"\begin{table}[ht!]")
    rows.append(r"\centering")
    rows.append(rf"\caption{{{caption}}}")
    rows.append(rf"\label{{{label}}}")
    rows.append(r"\resizebox{\textwidth}{!}{%")
    rows.append(r"\begin{tabular}{lrrrrrrrrr}")
    rows.append(r"\toprule")
    header = (
        r"\multicolumn{1}{c}{\textbf{Instance}} & "
        r"\multicolumn{1}{c}{\textbf{|V|}} & "
        r"\multicolumn{1}{c}{\textbf{|A|}} & "
        r"\multicolumn{1}{c}{\textbf{|B|}} & "
        r"\multicolumn{1}{c}{\textbf{Alpha}} & "
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Start\\ LB\end{tabular}}} & "
        r"\multicolumn{1}{c}{\textbf{LB}} & "
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Improve.\end{tabular}}} & "
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Att.\\ (s0)\end{tabular}}} & "
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Time (s)\end{tabular}}} \\"
    )
    rows.append(header)
    rows.append(r"\midrule")

    for idx, row in df.iterrows():
        inst = row.get("Instance", row.get("instance_name", ""))
        n = row.get("N", "-")
        m = row.get("M", "-")
        b = row.get("B", "-")
        alpha = row.get("Alpha", "-")
        start_ub = row.get("Start_LB", "-")
        lb = row.get("LB", "-")
        imp = row.get("improvement", row.get("Improvement"))
        att = row.get("attended_s0", row.get("Attended (s0)", "-"))
        time_ = row.get("Runtime", "-")
        if isinstance(alpha, float):
            alpha = _fmt_num(alpha, 1)
        if isinstance(start_ub, (int, float)):
            start_ub = _fmt_num(start_ub, 2)
        if isinstance(lb, (int, float)):
            lb = _fmt_num(lb, 2)
        if imp is not None:
            imp = _fmt_num(imp, 2)
        else:
            imp = "-"
        if isinstance(time_, (int, float)):
            time_ = _fmt_num(time_, 2)
        rowcolor = r"\rowcolor{rowgrey}" if use_rowcolor and (idx % 2 == 0) else (r"\rowcolor{rowlight}" if use_rowcolor else "")
        line = f"{rowcolor}{inst} & {n} & {m} & {b} & {alpha} & {start_ub} & {lb} & {imp} & {att} & {time_} \\\\"
        rows.append(line)
    rows.append(r"\bottomrule")
    rows.append(r"\end{tabular}%")
    rows.append(r"}")
    rows.append(r"\end{table}")
    return "\n".join(rows)


def parse_graph_file(filepath: str | Path) -> dict[str, Any]:
    """
    Parse a graph instance file. Returns dict with:
    N, M, B, cases_per_block (dict block_id -> cases), total_cases.
    """
    path = Path(filepath)
    if not path.is_file():
        return {}
    lines = path.read_text().splitlines()
    first = lines[0].split()
    N, M, B = int(first[0]), int(first[1]), int(first[2])
    cases_per_block: dict[int, int] = {}
    for ln in lines[1:]:
        parts = ln.split()
        if not parts:
            continue
        if parts[0] == "B" and len(parts) >= 3:
            block_id, cases = int(parts[1]), int(parts[2])
            cases_per_block[block_id] = cases
    total_cases = sum(cases_per_block.values())
    return {
        "N": N, "M": M, "B": B,
        "cases_per_block": cases_per_block,
        "total_cases": total_cases,
        "blocks_with_cases": len(cases_per_block),
    }


def parse_scenario_file(filepath: str | Path) -> dict[str, Any]:
    """
    Parse a scenarios file. Returns dict with:
    S, scenarios list of {probability, cases_per_block dict, total_cases}.
    """
    path = Path(filepath)
    if not path.is_file():
        return {}
    lines = path.read_text().splitlines()
    S = int(lines[0].strip())
    scenarios: list[dict[str, Any]] = [{"probability": 0.0, "cases_per_block": {}, "total_cases": 0} for _ in range(S)]
    for ln in lines[1:]:
        parts = ln.split()
        if not parts:
            continue
        if parts[0] == "P" and len(parts) >= 3:
            idx = int(parts[1])
            if idx < S:
                scenarios[idx]["probability"] = float(parts[2])
        elif parts[0] == "B" and len(parts) >= 4:
            idx, block_id, cases = int(parts[1]), int(parts[2]), int(parts[3])
            if idx < S:
                scenarios[idx]["cases_per_block"][block_id] = cases
    for sc in scenarios:
        sc["total_cases"] = sum(sc["cases_per_block"].values())
    return {"S": S, "scenarios": scenarios}


def load_all_instance_data(folders: list[str | Path]) -> dict[str, dict[str, Any]]:
    """
    Load graph + scenario data for all instances in the given folders.
    Returns dict keyed by instance_name (stem) -> {graph: {...}, scenarios: {...}}.
    """
    data: dict[str, dict[str, Any]] = {}
    for folder in folders:
        folder = Path(folder)
        if not folder.is_dir():
            continue
        for f in sorted(folder.glob("*.txt")):
            if f.name.startswith("scenarios"):
                continue
            inst_name = f.stem
            graph = parse_graph_file(f)
            scenario_file = folder / f"scenarios-{f.name}"
            scenarios = parse_scenario_file(scenario_file)
            data[inst_name] = {"graph": graph, "scenarios": scenarios}
    return data


def parse_greedy_result(filepath: str | Path) -> dict[str, Any] | None:
    """
    Parse a deterministic greedy heuristic result file.
    Returns dict with: N, M, B, LB (solution value), Route_Time, Attend_Time, Runtime,
    attended_blocks (count from Y), instance_name.
    """
    path = Path(filepath)
    if not path.is_file():
        return None
    data: dict[str, Any] = {"instance_name": path.stem, "attended_blocks": 0}
    for line in path.read_text().splitlines():
        parts = line.split(maxsplit=1)
        if len(parts) < 2:
            continue
        key, rest = parts[0].rstrip(":"), parts[1].strip()
        if key == "N":
            data["N"] = int(rest)
        elif key == "M":
            data["M"] = int(rest)
        elif key == "B":
            data["B"] = int(rest)
        elif key == "LB":
            data["LB"] = float(rest)
        elif key == "Runtime":
            data["Runtime"] = float(rest)
        elif key == "Route_Time":
            data["Route_Time"] = float(rest)
        elif key == "Attend_Time":
            data["Attend_Time"] = float(rest)
        elif key == "Y":
            data["attended_blocks"] = _count_attended_blocks(rest)
    data["att_pct"] = 100.0 * data["attended_blocks"] / data["B"] if data.get("B", 0) > 0 else 0
    return data


def parse_lagrangean_result(filepath: str | Path) -> dict[str, Any] | None:
    """
    Parse a Lagrangean relaxation result file.
    Returns dict with: N, M, B, LB, UB, Initial_LB, Initial_UB, Lambda, Max_Iter,
    Iterations, Improve_Iter, Reduction_factor, Runtime, gap_pct, instance_name.
    """
    path = Path(filepath)
    if not path.is_file():
        return None
    data: dict[str, Any] = {"instance_name": path.stem}
    for line in path.read_text().splitlines():
        parts = line.split(maxsplit=1)
        if len(parts) < 2:
            continue
        key, rest = parts[0].rstrip(":"), parts[1].strip()
        if key == "N":
            data["N"] = int(rest)
        elif key == "M":
            data["M"] = int(rest)
        elif key == "B":
            data["B"] = int(rest)
        elif key == "LB":
            data["LB"] = float(rest)
        elif key == "UB":
            data["UB"] = float(rest)
        elif key == "Initial_LB":
            data["Initial_LB"] = float(rest)
        elif key == "Initial_UB":
            data["Initial_UB"] = float(rest)
        elif key == "Lambda":
            data["Lambda"] = float(rest)
        elif key == "Max_Iter":
            data["Max_Iter"] = int(rest)
        elif key == "Iterations":
            data["Iterations"] = int(rest)
        elif key == "Improve_Iter":
            data["Improve_Iter"] = int(rest)
        elif key == "Reduction_factor":
            data["Reduction_factor"] = float(rest)
        elif key == "Runtime":
            data["Runtime"] = float(rest)
    if data.get("UB") and data["UB"] > 0 and data.get("LB") is not None:
        data["gap_pct"] = 100.0 * (data["UB"] - data["LB"]) / data["UB"]
    else:
        data["gap_pct"] = None
    return data


def parse_experiment_folder_greedy(base_dir: str | Path) -> list[tuple[str, dict[str, Any]]]:
    """Scan deterministic-results-greedy for experiment dirs and parse all .txt files."""
    base = Path(base_dir)
    if not base.is_dir():
        return []
    out: list[tuple[str, dict[str, Any]]] = []
    for exp_dir in sorted(base.iterdir()):
        if not exp_dir.is_dir() or not exp_dir.name.startswith("experiment-"):
            continue
        for f in sorted(exp_dir.glob("*.txt")):
            parsed = parse_greedy_result(f)
            if parsed:
                parsed["experiment"] = exp_dir.name
                out.append((exp_dir.name, parsed))
    return out


def parse_experiment_folder_lagrangean(base_dir: str | Path) -> list[tuple[str, dict[str, Any]]]:
    """Scan deterministic-results-lagrangean for experiment dirs and parse all .txt files."""
    base = Path(base_dir)
    if not base.is_dir():
        return []
    out: list[tuple[str, dict[str, Any]]] = []
    for exp_dir in sorted(base.iterdir()):
        if not exp_dir.is_dir() or not exp_dir.name.startswith("experiment-"):
            continue
        for f in sorted(exp_dir.glob("*.txt")):
            parsed = parse_lagrangean_result(f)
            if parsed:
                parsed["experiment"] = exp_dir.name
                out.append((exp_dir.name, parsed))
    return out


def parse_experiment_name_greedy(exp_name: str) -> dict[str, Any]:
    """Parse experiment-preproc-{use_preprocessing} into dict."""
    parts = exp_name.replace("experiment-preproc-", "")
    return {"use_preprocessing": int(parts)} if parts.isdigit() else {}


def parse_experiment_name_lagrangean(exp_name: str) -> dict[str, Any]:
    """Parse experiment-preproc-{prep}-heur-{heur}-barrier-{barrier} into dict."""
    # experiment-preproc-1-heur-0-barrier-1
    import re as _re
    m = _re.match(r"experiment-preproc-(\d+)-heur-(\d+)-barrier-(\d+)", exp_name)
    if not m:
        return {}
    return {
        "use_preprocessing": int(m.group(1)),
        "use_heuristic": int(m.group(2)),
        "use_barrier_method": int(m.group(3)),
    }


def export_greedy_to_latex(
    df, caption: str, label: str, use_rowcolor: bool = True,
) -> str:
    """Export greedy results to thesis-style LaTeX table."""
    rows: list[str] = []
    rows.append(r"\begin{table}[ht!]")
    rows.append(r"\centering")
    rows.append(rf"\caption{{{caption}}}")
    rows.append(rf"\label{{{label}}}")
    rows.append(r"\resizebox{\textwidth}{!}{%")
    rows.append(r"\begin{tabular}{lrrrrrrr}")
    rows.append(r"\toprule")
    rows.append(
        r"\multicolumn{1}{c}{\textbf{Instance}} & "
        r"\multicolumn{1}{c}{\textbf{|V|}} & "
        r"\multicolumn{1}{c}{\textbf{|A|}} & "
        r"\multicolumn{1}{c}{\textbf{|B|}} & "
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Attended\\ Blocks\end{tabular}}} & "
        r"\multicolumn{1}{c}{\textbf{LB}} & "
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Route\\ Time\end{tabular}}} & "
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Exec.\\ Time (s)\end{tabular}}} \\"
    )
    rows.append(r"\midrule")
    for idx, row in df.iterrows():
        inst = row.get("Instance", row.get("instance_name", ""))
        n = row.get("N", "-")
        m = row.get("M", "-")
        b = row.get("B", "-")
        att = row.get("attended_blocks", "-")
        lb = _fmt_num(row["LB"], 0) if isinstance(row.get("LB"), (int, float)) else "-"
        rt = _fmt_num(row["Route_Time"], 0) if isinstance(row.get("Route_Time"), (int, float)) else "-"
        time_ = _fmt_num(row["Runtime"], 2) if isinstance(row.get("Runtime"), (int, float)) else "-"
        rc = r"\rowcolor{rowgrey}" if use_rowcolor and (idx % 2 == 0) else (r"\rowcolor{rowlight}" if use_rowcolor else "")
        rows.append(f"{rc}{inst} & {n} & {m} & {b} & {att} & {lb} & {rt} & {time_} \\\\")
    rows.append(r"\bottomrule")
    rows.append(r"\end{tabular}%")
    rows.append(r"}")
    rows.append(r"\end{table}")
    return "\n".join(rows)


def export_lagrangean_to_latex(
    df, caption: str, label: str, use_rowcolor: bool = True,
) -> str:
    """Export Lagrangean relaxation results to thesis-style LaTeX table."""
    rows: list[str] = []
    rows.append(r"\begin{table}[ht!]")
    rows.append(r"\centering")
    rows.append(rf"\caption{{{caption}}}")
    rows.append(rf"\label{{{label}}}")
    rows.append(r"\resizebox{\textwidth}{!}{%")
    rows.append(r"\begin{tabular}{lrrrrrrrr}")
    rows.append(r"\toprule")
    rows.append(
        r"\multicolumn{1}{c}{\textbf{Instance}} & "
        r"\multicolumn{1}{c}{\textbf{|V|}} & "
        r"\multicolumn{1}{c}{\textbf{|A|}} & "
        r"\multicolumn{1}{c}{\textbf{|B|}} & "
        r"\multicolumn{1}{c}{\textbf{LB}} & "
        r"\multicolumn{1}{c}{\textbf{UB}} & "
        r"\multicolumn{1}{c}{\textbf{gap (\%)}} & "
        r"\multicolumn{1}{c}{\textbf{Iterations}} & "
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Exec.\\ Time (s)\end{tabular}}} \\"
    )
    rows.append(r"\midrule")
    for idx, row in df.iterrows():
        inst = row.get("Instance", row.get("instance_name", ""))
        n = row.get("N", "-")
        m = row.get("M", "-")
        b = row.get("B", "-")
        lb = _fmt_num(row["LB"], 2) if isinstance(row.get("LB"), (int, float)) else "-"
        ub = _fmt_num(row["UB"], 2) if isinstance(row.get("UB"), (int, float)) else "-"
        gap = _fmt_num(row.get("gap_pct"), 2) if row.get("gap_pct") is not None else "-"
        iters = row.get("Iterations", "-")
        time_ = _fmt_num(row["Runtime"], 2) if isinstance(row.get("Runtime"), (int, float)) else "-"
        rc = r"\rowcolor{rowgrey}" if use_rowcolor and (idx % 2 == 0) else (r"\rowcolor{rowlight}" if use_rowcolor else "")
        rows.append(f"{rc}{inst} & {n} & {m} & {b} & {lb} & {ub} & {gap} & {iters} & {time_} \\\\")
    rows.append(r"\bottomrule")
    rows.append(r"\end{tabular}%")
    rows.append(r"}")
    rows.append(r"\end{table}")
    return "\n".join(rows)


def parse_experiment_name_model(exp_name: str) -> dict[str, Any]:
    """Parse experiment-{alpha}-{model}-{model_type}-{use_preprocessing} into dict."""
    # experiment-0.1-TRAIL-MTZ-0
    parts = exp_name.replace("experiment-", "").split("-")
    if len(parts) < 4:
        return {}
    return {
        "alpha": float(parts[0]),
        "model": parts[1],
        "model_type": parts[2],
        "use_preprocessing": int(parts[3]),
    }


import math as _math


def export_lagrangean_combined_to_latex(
    df_all,
    config_order: list[str],
    config_labels: list[str],
    caption: str = "Results of Lagrangean Relaxations With Preprocessing.",
    label: str = "tab:results-lagrange-relax-preprocessing",
) -> str:
    """
    Export all 4 Lagrangean configs side-by-side in one LaTeX table.
    LB values use ceil, UB values use floor.

    config_order: list of experiment names in display order.
    config_labels: corresponding LaTeX header names (e.g. 'LR-RCSP-KN-Prep').
    """
    n_configs = len(config_order)
    instances = sorted(
        df_all["instance_name"].unique(),
        key=instance_sort_key,
    )

    # Build lookup: (config, instance) -> row dict
    lookup: dict[tuple[str, str], dict] = {}
    for _, row in df_all.iterrows():
        lookup[(row["experiment"], row["instance_name"])] = row.to_dict()

    rows: list[str] = []
    rows.append(r"\begin{table}[ht!]")
    rows.append(r"\centering")
    rows.append(rf"\caption{{{caption}}}")
    rows.append(rf"\label{{{label}}}")
    rows.append(r"\resizebox{\textwidth}{!}{%")

    col_spec = "lrrr|" + "|".join(["rrrrrr"] * n_configs)
    rows.append(rf"\begin{{tabular}}{{{col_spec}}}")
    rows.append(r"\toprule")

    # First header row: empty cols + config group headers
    hdr1_parts = [r"\multicolumn{1}{c}{}"] * 4
    for i, lbl in enumerate(config_labels):
        sep = "|" if i < n_configs - 1 else ""
        hdr1_parts.append(rf"\multicolumn{{6}}{{c{sep}}}{{\textbf{{{lbl}}}}}")
    rows.append(" &\n".join(hdr1_parts) + r" \\")

    # cmidrule
    cmidrules = []
    for i in range(n_configs):
        start = 5 + i * 6
        end = start + 5
        cmidrules.append(rf"\cmidrule(lr){{{start}-{end}}}")
    rows.append("".join(cmidrules))

    # Second header row
    base_cols = [
        r"\multicolumn{1}{c}{\textbf{Instance}}",
        r"\multicolumn{1}{c}{\textbf{|V|}}",
        r"\multicolumn{1}{c}{\textbf{|A|}}",
        r"\multicolumn{1}{c|}{\textbf{|B|}}",
    ]
    per_config_cols = [
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Initial\\ LB\end{tabular}}}",
        r"\multicolumn{1}{c}{\textbf{LB}}",
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Initial\\ UB\end{tabular}}}",
        r"\multicolumn{1}{c}{\textbf{UB}}",
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Gap\\ (\%)\end{tabular}}}",
    ]
    time_col_mid = r"\multicolumn{1}{c|}{\textbf{\begin{tabular}[c]{@{}c@{}}Time\\ (s)\end{tabular}}}"
    time_col_last = r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Time\\ (s)\end{tabular}}}"

    hdr2_parts = list(base_cols)
    for i in range(n_configs):
        hdr2_parts.extend(per_config_cols)
        if i < n_configs - 1:
            hdr2_parts.append(time_col_mid)
        else:
            hdr2_parts.append(time_col_last)
    rows.append(" &\n".join(hdr2_parts) + r" \\")
    rows.append(r"\midrule")

    def _ceil_int(v):
        if v is None or (isinstance(v, float) and _math.isnan(v)):
            return "-"
        return str(_math.ceil(v))

    def _floor_int(v):
        if v is None or (isinstance(v, float) and _math.isnan(v)):
            return "-"
        return str(_math.floor(v))

    def _gap_from_bounds(lb_raw, ub_raw):
        """Compute gap from ceil(LB) and floor(UB)."""
        if lb_raw is None or ub_raw is None:
            return "-"
        if isinstance(lb_raw, float) and _math.isnan(lb_raw):
            return "-"
        if isinstance(ub_raw, float) and _math.isnan(ub_raw):
            return "-"
        lb_c = _math.ceil(lb_raw)
        ub_f = _math.floor(ub_raw)
        if ub_f == 0:
            return "-"
        gap = 100.0 * (ub_f - lb_c) / ub_f
        return f"{gap:.2f}".replace(".", ",")

    def _time_str(v):
        if v is None or (isinstance(v, float) and _math.isnan(v)):
            return "-"
        return str(int(v))

    prev_city = None
    for inst in instances:
        city = "-".join(inst.split("-")[:-2])
        if prev_city is not None and city != prev_city:
            rows.append(r"\midrule")
        prev_city = city

        # Get N, M, B from the first config that has this instance
        n_val, m_val, b_val = "-", "-", "-"
        for cfg in config_order:
            d = lookup.get((cfg, inst))
            if d:
                n_val = d.get("N", "-")
                m_val = d.get("M", "-")
                b_val = d.get("B", "-")
                break

        parts = [str(inst), str(n_val), str(m_val), str(b_val)]

        for cfg in config_order:
            d = lookup.get((cfg, inst))
            if d:
                parts.append(_ceil_int(d.get("Initial_LB")))
                parts.append(_ceil_int(d.get("LB")))
                parts.append(_floor_int(d.get("Initial_UB")))
                parts.append(_floor_int(d.get("UB")))
                parts.append(_gap_from_bounds(d.get("LB"), d.get("UB")))
                parts.append(_time_str(d.get("Runtime")))
            else:
                parts.extend(["-"] * 6)

        rows.append(" & ".join(parts) + r" \\")

    rows.append(r"\bottomrule")
    rows.append(r"\end{tabular}%")
    rows.append(r"}")
    rows.append(r"\end{table}")
    return "\n".join(rows)


def parse_experiment_name_sa(exp_name: str) -> dict[str, Any]:
    """Parse experiment-{alpha}-{temp}-{temp_max}-{alpha_sa}-{max_iters}-{delta}-{first_improve}-{use_prep}."""
    # experiment-0.1-1.0-100-1.05-50-moderate-0-0
    parts = exp_name.replace("experiment-", "").split("-")
    if len(parts) < 8:
        return {}
    return {
        "alpha": float(parts[0]),
        "temperature": float(parts[1]),
        "temperature_max": int(parts[2]),
        "alpha_sa": float(parts[3]),
        "max_iters_sa": int(parts[4]),
        "delta_type": parts[5],
        "first_improve": int(parts[6]),
        "use_preprocessing": int(parts[7]),
    }
