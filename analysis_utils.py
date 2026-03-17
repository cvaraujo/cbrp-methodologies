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
    Returns dict with: N, M, B, S, Alpha, Start_UB, LB, Runtime, instance_name,
    attended_s0, attended_per_scenario, route_time_s0, attend_time_s0, improvement (Start_UB - LB).
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
            data["Start_UB"] = float(rest)
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

    if "Start_UB" in data and "LB" in data:
        data["improvement"] = data["Start_UB"] - data["LB"]
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
    df columns: Instance, |V|, |A|, |B|, Alpha, Start_UB, LB, Improvement, Attended (s0), Runtime.
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
        r"\multicolumn{1}{c}{\textbf{\begin{tabular}[c]{@{}c@{}}Start\\ UB\end{tabular}}} & "
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
        start_ub = row.get("Start_UB", "-")
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
