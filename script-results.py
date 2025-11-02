import pandas as pd
import xlsxwriter
import os, sys


class Instance:
    def __init__(self, name):
        self.name = name.split(".")[0]
        self.nodes = ""
        self.arcs = ""
        self.blocks = ""
        self.scenarios = 0
        self.alpha = 0.8
        self.initial_lb = 0.0
        self.lb = 0.0
        self.runtime = 0.0
        self.route_time = 0.0
        self.attend_time = 0.0
        self.temperature = 0.0
        self.T = 0
        self.temperature_max = 0.0
        self.alpha_sa = 0.0
        self.max_iters_sa = 0
        self.delta_type = "moderated"
        self.improve_used = "best_improve"
        self.attend_blocks = 0

    def to_list(self):
        return pd.Series(
            data={
                "Instance": self.name,
                "|V|": self.nodes,
                "|A|": self.arcs,
                "|B|": self.blocks,
                "|S|": self.scenarios,
                "Alpha": str(self.alpha),
                "T": str(self.T),
                "Initial LB": str(self.initial_lb),
                "LB": str(self.lb),
                "Temperature": str(self.temperature),
                "Temperature Max": str(self.temperature_max),
                "Alpha SA": str(self.alpha_sa),
                "Max Iters SA": str(self.max_iters_sa),
                "Delta Type": self.delta_type,
                "Improve Used": (
                    "first_improve" if self.improve_used else "best_improve"
                ),
                "Route Time": str(self.route_time),
                "Attend Time": str(self.attend_time),
                "Time (s)": str(self.runtime),
            }
        )


def get_result(folder_name) -> [Instance]:  # type: ignore
    instances = os.listdir(folder_name)
    results = []
    folders_infos = folder_name.split("-")

    for i in instances:
        f = open(os.path.join(folder_name, i), "r")
        lines = f.readlines()
        i_splitted = i.split("-")
        if len(i_splitted) < 2:
            inst = Instance("SBRP-" + i)
        else:
            if "limoeiro" == i_splitted[1]:
                i = i.replace("limoeiro", "limoeiro-norte")
            inst = Instance(i)

        inst.T = 1200
        inst.temperature = float(folders_infos[1])
        inst.temperature_max = float(folders_infos[2])
        inst.alpha_sa = float(folders_infos[3])
        inst.max_iters_sa = int(folders_infos[4])
        inst.delta_type = folders_infos[5]
        inst.improve_used = int(folders_infos[6])

        try:
            for l in lines:
                content = l.strip().split(" ")
                if content[0] == "N:":
                    inst.nodes = content[1]
                elif content[0] == "M:":
                    inst.arcs = content[1]
                elif content[0] == "B:":
                    inst.blocks = content[1]
                elif content[0] == "S:":
                    inst.scenarios = int(content[1])
                elif content[0] == "Alpha:":
                    inst.alpha = float(content[1])
                elif content[0] == "Start_UB:":
                    inst.initial_lb = float(content[1])
                elif content[0] == "LB:":
                    inst.lb = float(content[1])
                elif content[0] == "Runtime:":
                    inst.runtime = float(content[1])
                elif content[0] == "Y:":
                    inst.attended_blocks = len(content[1].split(","))
                elif content[0] == "Route_Time:":
                    inst.route_time = float(content[1])
                elif content[0] == "Attend_Time:":
                    inst.attend_time = float(content[1])
        except:
            print(i)
        results.append(inst)

    return results


writer = pd.ExcelWriter(
    "results-stochastic-simulated-annealing.xlsx", engine="xlsxwriter"
)

ROOT_DIR = "."

final_df = pd.DataFrame(
    columns=[
        "Instance",
        "|V|",
        "|A|",
        "|B|",
        "|S|",
        "Alpha",
        "T",
        "Initial LB",
        "LB",
        "Temperature",
        "Temperature Max",
        "Alpha SA",
        "Max Iters SA",
        "Delta Type",
        "Improve Used",
        "Route Time",
        "Attend Time",
        "Time (s)",
    ]
)

results = []
for root, dirs, files in os.walk(ROOT_DIR):
    for dir_name in dirs:
        if dir_name.split("-")[0] != "experiment":
            continue
        print(f"Processing {dir_name}...")
        results.extend(get_result(os.path.join(root, dir_name)))

for res in results:
    final_df.loc[len(final_df) - 1] = res.to_list()

for col in final_df.select_dtypes(include="float").columns:
    final_df[col] = final_df[col].apply(lambda x: f"{x:.3f}".replace(".", ","))

# print(final_df)

final_df.to_excel(writer, sheet_name="sa-results")
writer.close()

# Create Excel writer
# for exp_name, exp_data in experiments.items():
#     # Create DataFrame from results
#     df = pd.DataFrame(exp_data["results"])
#     # Add parameters as columns
#     for key, value in exp_data["params"].items():
#         df[key] = value
#     # Write to a sheet named after the experiment
#     df.to_excel(writer, sheet_name=exp_name[:31], index=False)  # sheet names max 31 chars
