import os
import subprocess
from pathlib import Path

folders = ["instances/simulated-alto-santo", "instances/simulated-limoeiro"]
commands = []

Path("stochastic-results-sa").mkdir(parents=True, exist_ok=True)
for temperature in [1.0, 5.0, 10.0]:
    for temperature_max in [100, 300, 500, 1000]:
        for alpha in [1.05, 1.10, 1.15, 1.20, 1.25]:
            for max_iters in [50, 100, 200]:
                for delta_type in ["moderate", "weak"]:
                    for first_improve in [0, 1]:
                        output_folder = f"stochastic-results-sa/experiment-{temperature}-{temperature_max}-{alpha}-{max_iters}-{delta_type}-{first_improve}"
                        Path(output_folder).mkdir(parents=True, exist_ok=True)

                        for folder in folders:
                            instance = os.listdir(folder)
                            for inst in instance:
                                if inst.split("-")[0] == "scenarios":
                                    continue
                                graph = f"{folder}/{inst}"
                                scenarios = f"{folder}/scenarios-{inst}"
                                command = f"./cbrp {graph} {scenarios} {output_folder}/{inst} 1200 {temperature} {temperature_max} {alpha} {max_iters} {delta_type} {first_improve}"
                                commands.append(command)

for c in commands:
    print(c)
    p = subprocess.Popen(c, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    msg, err = p.communicate()
    if msg:
        print(msg)
    print("OK!!")
