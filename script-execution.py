import gc
import os
import shlex
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, as_completed
from pathlib import Path

def RunSimulatedAnnealing(folders: list[str], output_folder: str) -> list[str]:
    commands = []
    Path(output_folder).mkdir(parents=True, exist_ok=True)
    for alpha in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]:
        for temperature in [1.0, 5.0, 10.0]:
            for temperature_max in [100, 300, 500, 1000]:
                for alpha_sa in [1.05, 1.10, 1.15, 1.20, 1.25]:
                    for max_iters_sa in [50, 100, 200]:
                        for delta_type in ["moderate", "weak"]:
                            for first_improve in [0, 1]:
                                for use_preprocessing in [0, 1]:
                                    new_output_folder = f"{output_folder}/experiment-{alpha}-{temperature}-{temperature_max}-{alpha_sa}-{max_iters_sa}-{delta_type}-{first_improve}-{use_preprocessing}"
                                    Path(new_output_folder).mkdir(parents=True, exist_ok=True)

                                    for folder in folders:
                                        instance = os.listdir(folder)
                                        for inst in instance:
                                            if inst.split("-")[0] == "scenarios":
                                                continue
                                            graph = f"{folder}/{inst}"
                                            scenarios = f"{folder}/scenarios-{inst}"
                                            command = f"./cbrp-stoc-sa {graph} {scenarios} {new_output_folder}/{inst} {alpha} {temperature} {temperature_max} {alpha_sa} {max_iters_sa} {delta_type} {first_improve} {use_preprocessing}"
                                            commands.append(command)
    return commands

def RunGreedyHeuristic(folders: list[str], output_folder: str) -> list[str]:
    """cbrp-det-greedy: file_graph result_file use_preprocessing"""
    commands = []
    Path(output_folder).mkdir(parents=True, exist_ok=True)
    for use_preprocessing in [1]:
        new_output_folder = f"{output_folder}/experiment-preproc-{use_preprocessing}"
        Path(new_output_folder).mkdir(parents=True, exist_ok=True)
        for folder in folders:
            instance = os.listdir(folder)
            for inst in instance:
                if inst.split("-")[0] == "scenarios":
                    continue
                graph = f"{folder}/{inst}"
                command = f"./cbrp-det-greedy {graph} {new_output_folder}/{inst} {use_preprocessing}"
                commands.append(command)
    return commands

def RunLagrangean(folders: list[str], output_folder: str) -> list[str]:
    """cbrp-det-lagr: file_graph result_file use_preprocessing use_heuristic use_barrier_method"""
    commands = []
    Path(output_folder).mkdir(parents=True, exist_ok=True)
    for use_preprocessing in [1]:
        for use_heuristic in [0, 1]:
            for use_barrier_method in [0, 1]:
                new_output_folder = f"{output_folder}/experiment-preproc-{use_preprocessing}-heur-{use_heuristic}-barrier-{use_barrier_method}"
                Path(new_output_folder).mkdir(parents=True, exist_ok=True)
                for folder in folders:
                    instance = os.listdir(folder)
                    for inst in instance:
                        if inst.split("-")[0] == "scenarios":
                            continue
                        graph = f"{folder}/{inst}"
                        command = f"./cbrp-det-lagr {graph} {new_output_folder}/{inst} {use_preprocessing} {use_heuristic} {use_barrier_method}"
                        commands.append(command)
    return commands

def RunDeterministicModel(folders: list[str], output_folder: str) -> list[str]:
    commands = []
    Path(output_folder).mkdir(parents=True, exist_ok=True)
    for model in ["MTZ", "EXP"]:
        for model_type in ["TRAIL", "WALK"]:
            for use_preprocessing in [1]:
                for use_frac_cut in [0, 1]:
                        new_output_folder = f"{output_folder}/experiment-{model}-{model_type}-{use_preprocessing}-{use_frac_cut}"
                        Path(new_output_folder).mkdir(parents=True, exist_ok=True)
                        for folder in folders:
                            instance = os.listdir(folder)
                            for inst in instance:
                                if inst.split("-")[0] == "scenarios":
                                    continue
                                graph = f"{folder}/{inst}"
                                command = f"./cbrp-det {graph} {model} {new_output_folder}/{inst} {model_type} {use_preprocessing} {use_frac_cut} 0"
                                commands.append(command)
    return commands

def RunStochasticModel(folders: list[str], output_folder: str) -> list[str]:
    commands = []
    Path(output_folder).mkdir(parents=True, exist_ok=True)
    for alpha in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]:
        for model_type in ["MTZ", "EXP"]:
            for model in ["TRAIL", "WALK"]:
                for use_preprocessing in [0, 1]:
                    new_output_folder = f"{output_folder}/experiment-{alpha}-{model_type}-{model}-{use_preprocessing}"
                    Path(new_output_folder).mkdir(parents=True, exist_ok=True)
                    for folder in folders:
                        instance = os.listdir(folder)
                        for inst in instance:
                            if inst.split("-")[0] == "scenarios":
                                continue
                            graph = f"{folder}/{inst}"
                            scenarios = f"{folder}/scenarios-{inst}"
                            command = f"./cbrp-stoc {graph} {scenarios} {new_output_folder}/{inst} {model_type} {model} {alpha} {use_preprocessing} 0 0"
                            commands.append(command)
    return commands

def run_command(cmd: str) -> tuple[str, str | None]:
    """Run a single command in a clean process (no shell). Returns (command, stdout or None)."""
    args = shlex.split(cmd)
    p = subprocess.Popen(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd=os.getcwd(),
        env=os.environ.copy(),
    )
    out, _ = p.communicate()
    result = (cmd, out.decode(errors="replace") if out else None)
    del out
    return result

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script-execution.py <mode>")
        print("Mode:")
        print("  - SA: Run simulated annealing experiments")
        print("  - MODEL: Run Stochastic Model experiments")
        print("  - DET: Run Deterministic Model experiments")
        print("  - GREEDY: Run deterministic greedy heuristic")
        print("  - LAGR: Run Lagrangean relaxation")
        sys.exit(1)

    mode = sys.argv[1]
    # Metodologias determinísticas: instâncias em cases-* (sem cenários)
    # Metodologias estocásticas: instâncias em simulated-* (com cenários)
    folders_det = ["instances/cases-alto-santo", "instances/cases-limoeiro"]
    folders_stoc = ["instances/simulated-alto-santo", "instances/simulated-limoeiro"]
    commands = []
    if mode == "SA":
        commands = RunSimulatedAnnealing(folders_stoc, "stochastic-results-sa")
    elif mode == "MODEL":
        commands = RunStochasticModel(folders_stoc, "stochastic-results-model")
    elif mode == "DET":
        commands = RunDeterministicModel(folders_det, "deterministic-results")
    elif mode == "GREEDY":
        commands = RunGreedyHeuristic(folders_det, "deterministic-results-greedy")
    elif mode == "LAGR":
        commands = RunLagrangean(folders_det, "deterministic-results-lagrangean")
    else:
        print("Invalid mode")
        sys.exit(1)

    n_cpus = os.cpu_count() or 4
    max_workers = n_cpus if mode in ("SA", "GREEDY", "LAGR") else 1
    print(f"Running with {max_workers} workers (CPUs: {n_cpus})")
    if mode in ("SA", "GREEDY", "LAGR"):
        # ProcessPoolExecutor: cada worker é um processo; ao terminar, o SO libera toda a memória
        # do binário C++, sem acumular no processo Python principal.
        with ProcessPoolExecutor(max_workers=max_workers) as executor:
            futures = {executor.submit(run_command, c): c for c in commands}
            for future in as_completed(futures):
                cmd, msg = future.result()
                print(cmd)
                if msg:
                    print(msg)
                print("OK!!")
                del msg
                gc.collect()
    else:
        for c in commands:
            print(c)
            args = shlex.split(c)
            p = subprocess.Popen(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                cwd=os.getcwd(),
                env=os.environ.copy(),
            )
            msg, _ = p.communicate()
            if msg:
                print(msg.decode(errors="replace"))
            del msg
            gc.collect()
            print("OK!!")