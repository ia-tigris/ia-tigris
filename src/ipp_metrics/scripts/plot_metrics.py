import argparse
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
from pathlib import Path
import scipy.stats as stats

COLORS = [[230, 25, 75],   [60, 180, 75],   [255, 225, 25], [0, 130, 200],
               [245, 130, 48],  [145, 30, 180],  [70, 240, 240], [240, 50, 230],
               [210, 245, 60],  [250, 190, 212], [0, 128, 128],  [220, 190, 255],
               [170, 110, 40],  [255, 250, 200], [128, 0, 0],    [170, 255, 195],
               [128, 128, 0],   [255, 215, 180], [0, 0, 128],    [128, 128, 128],
               [255, 255, 255], [0, 0, 0]]

def plot_individual_vars(indVarFileName, showPlot=False, savePlot=True):
    global COLORS
    data = pd.read_csv(indVarFileName)
    cols = len(data.columns)
    colString = "variance"
    data_list = []
    plt.figure()
    for i in range(cols-1):
        col = data[colString + str(i)].tolist()
        data_list.append([col])
        color_list = COLORS[(i + 1) % len(COLORS)]
        color_tuple = tuple(c / 255.0 for c in color_list)
        plt.plot(col, label="Variance of tracker" + str(i + 1), color=color_tuple)
    planner_name = indVarFileName.split('_')[1]
    plt.title("Variance for "  + planner_name)
    plt.xlabel('Time')
    plt.ylabel('Variance')
    plt.legend()
    if savePlot:
        plt.savefig(indVarFileName + ".png")
    if showPlot:
        plt.show()

def plot_total_vars(totVarFileName, showPlot=False, savePlot=True):
    data = pd.read_csv(totVarFileName)
    colString = "variance"
    data_list = []
    plt.figure()
    col = data[colString].tolist()
    data_list.append([col])
    plt.plot(col, label="Total Variance")
    planner_name = totVarFileName.split('_')[1]
    plt.title("Variance for " + planner_name)
    plt.xlabel('Time')
    plt.ylabel('Variance')
    plt.legend()
    if savePlot:
        plt.savefig(totVarFileName + ".png")
    if showPlot:
        plt.show()

def plot_multiple_runs_total_vars(totVarBaseFileName):
    # print("TBD")


def plot_std_dev(X1, planner, seq="", showPlot=True, savePlot=False, fig=None, ax=None, index=None):
    t = np.arange(X1.shape[1])
    mu1 = X1.mean(axis=0)
    sigma1 = X1.std(axis=0)
    if fig == None and ax == None:
        fig, ax = plt.subplots(1)
    if index == None:
        ax.plot(t, mu1.flatten(), lw=2, label='Mean Variance', color='blue')
        ax.fill_between(t, mu1+sigma1, mu1-sigma1, facecolor='blue', alpha=0.5)
    else:
        global COLORS
        color_list = COLORS[(index + 1) % len(COLORS)]
        color_tuple = tuple(c / 255.0 for c in color_list)
        ax.plot(t, mu1.flatten(), lw=2, label='Mean Variance of tracker' + str(index + 1), color=color_tuple)
        ax.fill_between(t, mu1+sigma1, mu1-sigma1, facecolor=color_tuple, alpha=0.5)
    #ax.set_ylim([0.54e8, 0.56e8])
    ax.set_title('Average Variance for ' + planner)
    ax.set_xlabel('Time')
    ax.set_ylabel('Variance')
    ax.legend()
    if savePlot:
        plt.savefig(planner + "_" + seq + "variance.png")
    if showPlot:
        plt.show()

def plot_std_dev_all(planner_results, planner_names, showPlot=True, savePlot=True): 
    fig, ax = plt.subplots(1)
    for i in range(len(planner_results)):
        if planner_results[i].shape[0] != 0:
            if planner_results[i].shape.__len__() == 1:
                t = np.arange(planner_results[i].shape[0])
                mu1 = planner_results[i]
                sigma1 = planner_results[i]
                ax.plot(t, mu1.flatten(), lw=2, label=planner_names[i])
                ax.fill_between(t, mu1+sigma1, mu1-sigma1, alpha=0.5)
            else:
                t = np.arange(planner_results[i].shape[1])
                mu1 = planner_results[i].mean(axis=0)
                sigma1 = planner_results[i].std(axis=0)
                ax.plot(t, mu1.flatten(), lw=2, label=planner_names[i])
                ax.fill_between(t, mu1+sigma1, mu1-sigma1, alpha=0.5)
        # print("Mean and variance for " + planner_names[i] + " is " + str(mu1[450]) + " and " + str(sigma1[450]))

    #ax.set_ylim([0.54e8, 0.56e8])
    ax.set_title('Mean Variance for planners')
    ax.set_xlabel('Time')
    ax.set_ylabel('Variance')
    ax.legend()
    if savePlot:
        plt.savefig(planner + "_" + seq + "variance.png")
    if showPlot:
        plt.show()

def plot_conf_interval_all(planner_results, planner_names, showPlot=True, savePlot=False):
    fig, ax = plt.subplots(1)
    
    if len(planner_results) > 2:
        # print("Can only compar two planners for this")
        return
    alpha = .05
    data1 = planner_results[0]
    data2 = planner_results[1]
    min_length = min(data1.shape[1], data2.shape[1])
    num_samples = data1.shape[0]
    diff_samples = data1[:,0:min_length] - data2[:,0:min_length]
    mean_diff = diff_samples.mean(axis=0)
    var_diff = diff_samples.var(axis=0, ddof=1)
    critical_value = stats.t.ppf(q=1-alpha/2, df = num_samples-1)
    rad = critical_value * np.sqrt(var_diff)/np.sqrt(num_samples)
    t = np.arange(min_length)
    ax.plot(t, mean_diff.flatten(), lw=2)
    ax.fill_between(t, mean_diff+rad, mean_diff-rad, alpha=0.5)
    # mean1 = planner_results[0].mean(axis=0)
    # mean2 = planner_results[1].mean(axis=0)
    # std1 = planner_results[0].std(axis=0)
    # std2 = planner_results[1].std(axis=0)
    # min_length = min(mean1.shape[0], mean2.shape[0])
    # diff = mean1[0:min_length] - mean2[0:min_length]


    #ax.set_ylim([0.54e8, 0.56e8])
    ax.set_title('Confidence Interval of Variance for planners')
    ax.set_xlabel('Time')
    ax.set_ylabel('Mean Variance')
    ax.legend()
    if savePlot:
        plt.savefig(planner + "_" + seq + "variance.png")
    if showPlot:
        plt.show()

# Save the results of all runs in a (N, M) matrix, 
# where N is the number of runs and M is the number of time steps
def read_experiments(rootdir, baseStr):
    count = 0
    data = np.array([])
    for path in Path(rootdir).glob(baseStr):
        if "individual" not in str(path):
            # print("Match: " + str(path))
            count += 1
            csv = pd.read_csv(str(path))
            var_list = csv["variance"].to_list()
            if len(var_list) < 300:
                # print("--Skipping " + str(path))
                continue
            if len(data) == 0:
                data = np.array(var_list)
            else:
                if (data.shape.__len__() == 1):
                    min_colomn_size = min(data.shape[0], len(var_list))
                    var_arr = np.array(var_list[0:min_colomn_size])
                    data = np.vstack((data[0:min_colomn_size], var_arr))
                else:
                    min_colomn_size = min(data.shape[1], len(var_list))
                    var_arr = np.array(var_list[0:min_colomn_size])
                    data = np.vstack((data[:,0:min_colomn_size], var_arr))
                
    # print(baseStr)
    ave_var = data[:,0:380].mean(axis=1)
    # print("Number of experiments: " + str(len(ave_var)))
    # print("Average Average Variance: " + str(ave_var.mean(axis=0)) + " and std: " + str(ave_var.std(axis=0)))  
    # print(str(data.shape[0]) + " out of " + str(count) + " experiments were used")
    # print("Final data shape: " + str(data.shape) + "\n")
    return data

def process_ind_var_experiments(rootdir, baseStr, planner, seq="", showPlot=True, savePlot=False):
    data = np.array([])
    tracker = 0
    fig, ax = plt.subplots(1)
    try:
        while True:
            # print(str(tracker))
            for path in Path(rootdir).glob(baseStr):
                # print("match: " + str(path))
                csv = pd.read_csv(str(path))
                var_list = csv["variance" + str(tracker)].to_list()
                if len(data) == 0:
                    data = np.array(var_list)
                else:
                    var_arr = np.array(var_list)
                    data = np.vstack((data, var_arr))
            if len(data) != 0:
                plot_std_dev(data, planner, False, fig, ax, tracker)
            tracker = tracker + 1
    except KeyError:
        if savePlot:
            plt.savefig(planner + "_" + seq + "individual_variance.png")
        if showPlot:
            plt.show()

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Plot metrics')
    parser.add_argument("--rootdir", type=str, default=".", help="root directory")
    parser.add_argument("--planner", type=str, default="rimcts", help="planner name")
    parser.add_argument("--combine_all_planners", type=bool, default=False, help="bool")
    parser.add_argument("--seq", type=str, default="*", help="sequence name")
    parser.add_argument("--singlerun", type=str, default="", help="plot a single experiment")

    args = parser.parse_args()

    planner = args.planner
    seq = args.seq
    # os.chdir(os.path.expanduser('~/.ros/friday_night_tests/'))
    os.chdir(os.path.expanduser('~/.ros'))
    if args.combine_all_planners:
        planner_names = ["random", "greedy", "rimcts", "rpmcts", "limcts", "lpmcts"]
        planner_names = ["random", "greedy", "rimcts", "rpmcts", "limcts"]
        # planner_names = ["rimcts", "rpmcts", "limcts"]
        # planner_names = ["greedy", "limcts"]
        planner_results = []
        fig, ax = plt.subplots(1)

        for planner in planner_names:
            data = read_experiments(args.rootdir, '*_' + planner + '_seq_' + seq + '_variance.csv')
            planner_results.append(data)
        plot_std_dev_all(planner_results, planner_names, showPlot=True, savePlot=False) 
        # plot_conf_interval_all(planner_results, planner_names, showPlot=True, savePlot=False)  
    elif args.singlerun == "":
        total_var_exps = read_experiments(args.rootdir, '*_' + planner + '_seq_' + seq + '_variance.csv')
        plot_std_dev(total_var_exps, planner)
        process_ind_var_experiments('.', '*_' + planner + '_seq_' + seq + '_individual_variance.csv', planner)
    else:
        plot_total_vars(args.singlerun + "_variance.csv")
        plot_individual_vars(args.singlerun + "_individual_variance.csv")