import os
import subprocess
from pathlib import Path

folders = ["instances/cases-alto-santo", "instances/cases-limoeiro"]
commands = []

Path("deterministic-results-lr").mkdir(parents=True, exist_ok=True)
for use_preprocessing in [0, 1]:
    for use_heuristic in [0, 1]:
        for use_barrier_method in [0, 1]:
            for folder in folders:
                output_folder = f"results-lr/{folder.split('/')[-1]}-p{use_preprocessing}-h{use_heuristic}-b{use_barrier_method}"
                Path(output_folder).mkdir(parents=True, exist_ok=True)
                instance = os.listdir(folder)
                for inst in instance:
                    if inst.split("-")[0] == "scenarios":
                        continue
                    graph = f"{folder}/{inst}"
                    command = f"./cbrp-det-lag {graph} {output_folder}/{inst} {use_preprocessing} {use_heuristic} {use_barrier_method}"
                    commands.append(command)

for c in commands:
    print(c)
    p = subprocess.Popen(c, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    msg, err = p.communicate()
    if msg:
        print(msg)
    print("OK!!")
