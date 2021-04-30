import numpy as np
from tabulate import tabulate
num_pts = 10000

def parse_data(alg_name, target_np):
    means = np.mean(target_np, axis=0)
    time_set = round(means[0], 5)
    time_iter = round(means[1] * 1000, 5)
    time_total = round(np.mean(np.sum(target_np, axis=1)), 5)

    return [alg_name, time_set, time_iter, time_total]

if __name__ == "__main__":
    TARGET = "knn" #"knn" or radius
    k_pts = [1, 10, 100, 500, 1000, 5000]
    radiuses = [5, 10, 20, 40, 80]
    if TARGET == "knn":
        target_param = k_pts
        table_header = ["K points", "Alg.", "Init. [s]", "Search [ms]", "Total [s]"]
    elif TARGET == "radius":
        target_param = radiuses
        table_header = ["radius [m]", "Alg.", "Init. [s]", "Search [ms]", "Total [s]"]
    for num_pts in [10000, 30000, 60000, 100000, 200000]:
        print("Num. points: " + str(num_pts))
        table_line = []
        for i in target_param:
            pcl = np.loadtxt(TARGET + "/pcl_" + str(i) + "_" + str(num_pts)+".txt", delimiter=" ")
            nano = np.loadtxt(TARGET + "/nano_" + str(i) + "_" + str(num_pts)+".txt", delimiter=" ")
            pico = np.loadtxt(TARGET + "/pico_" + str(i) + "_" + str(num_pts)+".txt", delimiter=" ")
            table_viz = []
            table_viz.append(parse_data("PCL", pcl))
            table_viz.append(parse_data("Nano", nano))
            table_viz.append(parse_data("Pico", pico))
            per_line = [i]
            for j in range(4):
                cell = ""
                for k in range(3):
                    cell += str(table_viz[k][j])
                    if k != 2:
                        cell += "\n"

                per_line.append(cell)

            table_line.append(per_line)

        print(tabulate(table_line, headers=table_header, tablefmt="fancy_grid"))
