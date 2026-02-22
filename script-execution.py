import os, subprocess, sys
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

def RunStochasticModel(folders: list[str], output_folder: str) -> list[str]:
    commands = []
    Path(output_folder).mkdir(parents=True, exist_ok=True)
    for alpha in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]:
        for model in ["TRAIL", "WALK"]:
            for model_type in ["MTZ", "EXP"]:
                for use_preprocessing in [0, 1]:
                    new_output_folder = f"{output_folder}/experiment-{alpha}-{model}-{model_type}-{use_preprocessing}"
                    Path(new_output_folder).mkdir(parents=True, exist_ok=True)
                    for folder in folders:
                        instance = os.listdir(folder)
                        for inst in instance:
                            if inst.split("-")[0] == "scenarios":
                                continue
                            graph = f"{folder}/{inst}"
                            scenarios = f"{folder}/scenarios-{inst}"
                            command = f"./cbrp-stoc {graph} {scenarios} {new_output_folder}/{inst} {model} {model_type} {alpha} {use_preprocessing} 0 0"
                            commands.append(command)
    return commands

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script-execution.py <mode>")
        print("Mode:")
        print("  - SA: Run simulated annealing experiments")
        print("  - MODEL: Run Stochastic Model experiments")
        sys.exit(1)
    
    folders = ["instances/simulated-alto-santo", "instances/simulated-limoeiro"]
    commands = []
    mode = sys.argv[1]
    if mode == "SA":
        commands = RunSimulatedAnnealing(folders, "stochastic-results-sa")
    elif mode == "MODEL":
        commands = RunStochasticModel(folders, "stochastic-results-model")
    else:
        print("Invalid mode")
        sys.exit(1)

    for c in commands:
        print(c)
        p = subprocess.Popen(c, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        msg, err = p.communicate()
        if msg:
            print(msg)
        print("OK!!")