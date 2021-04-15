import numpy as np

num_pts = 200000
def disp_outputs(alg_name, target_np):
    means = np.mean(target_np, axis=0)
    # print(means[0], means[1] * 1000, np.mean(np.sum(target_np, axis=1)))
    print("& " + alg_name + " & "+ str(round(means[0], 5)) +" & " + str(round(means[1] * 1000, 5)) + " & " + str(round(np.mean(np.sum(target_np, axis=1)), 5)) + " \\" + "\\")

TARGET = "radius" #"nearest" or radius
# for i in [1, 10, 500, 1000, 5000]:
for i in [1, 10, 20, 40, 80]:
    pcl = np.loadtxt(TARGET + "/pcl_" + str(i) + "_" + str(num_pts)+".txt", delimiter=" ")
    nano = np.loadtxt(TARGET + "/nano_" + str(i) + "_" + str(num_pts)+".txt", delimiter=" ")
    pico = np.loadtxt(TARGET + "/pico_" + str(i) + "_" + str(num_pts)+".txt", delimiter=" ")
    disp_outputs("PCL", pcl)
    disp_outputs("Nano", nano)
    disp_outputs("Pico", pico)
    print("--------------")
