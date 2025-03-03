import os
import subprocess
from pathlib import Path

folders = ["instances/cases-alto-santo", "instances/cases-limoeiro"]
algorithms = ["EXP", "MTZ"]
folders_outp = [
            "results/results-trail-mtz",
            "results/results-trail-mtz-prep",
            "results/results-trail-exp",
            "results/results-trail-exp-prep",
            "results/results-trail-exp-frac-cut",
            "results/results-trail-exp-frac-cut-prep",
        ]
commands = []

for direc in folders_outp:
    Path(direc).mkdir(parents=True, exist_ok=True)
    
for f in folders:
    instance = os.listdir(f)
    for inst in instance:
        for alg in algorithms:
            graph = f + "/" + inst
            scenarios = '\"\"'
            commands.append("./cbrp " + graph + " " + scenarios + " " + "results/results-trail-mtz/" + inst + " MTZ TRAIL 1200 0 0")
            commands.append("./cbrp " + graph + " " + scenarios + " " + "results/results-trail-mtz-prep/" + inst + " MTZ TRAIL 1200 1 0")

            commands.append("./cbrp " + graph + " " + scenarios + " " + "results/results-trail-exp/" + inst + " EXP TRAIL 1200 0 0")
            commands.append("./cbrp " + graph + " " + scenarios + " " + "results/results-trail-exp-prep/" + inst + " EXP TRAIL 1200 1 0")

            commands.append("./cbrp " + graph + " " + scenarios + " " + "results/results-trail-exp-frac-cut/" + inst + " EXP TRAIL 1200 0 1")
            commands.append("./cbrp " + graph + " " + scenarios + " " + "results/results-trail-exp-frac-cut-prep/" + inst + " EXP TRAIL 1200 1 1")
            
for c in commands:
    print(c)
    p = subprocess.Popen(c, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    msg, err = p.communicate()
    if msg:
        print(msg)
    print("OK!!")
