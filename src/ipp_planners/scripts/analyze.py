import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

def loop_over_files_and_plot(folder_path, line_color, ax, column_name='info'):
    xvals = np.linspace(0, 10, 1000)
    ysum  = np.zeros(1000)
    ycount = 0
    
    for file in os.listdir(folder_path):
        # import csv file and plot
        if 'plan' in file:
            file_path = os.path.join(folder_path, file)
            data = pd.read_csv(file_path, sep=',') 
            for column in data:
                if column == 'time':
                    time = data[column]
                elif column_name in column:
                    # check if column is empty or too large
                    if data[column].empty:
                        print('Skipping empty: ', file)
                        continue
                    if np.max(data[column]) > 9999999:
                        print('Skipping file big num: ', file)
                        continue
                    if np.min(data[column]) < -9999999:
                        print('Skipping file: ', file)
                        
                        continue
                    yinterp = np.interp(xvals, time, data[column])  
                    ysum += yinterp
                    ycount += 1
                    
                    ax = data.plot(ax=ax, x='time', y=column, color=line_color, linewidth=1, alpha=0.2)
    # plot average
    ax.plot(xvals, ysum/ycount, color=line_color, label='Average', linewidth=3)

def plot_mem_cpu(data, column_name='mem'):
    fig, ax = plt.subplots(figsize=(10,4))
    # loop over columns of pandas dataframe
    for column in data:
        if column == 'time':
            time = data[column]
        # elif 'cpu' in column:
        #     plot_cpu_usage(time, data[column], column)
        elif column_name in column:
            print(column)
            # check if the column is empty
            if 'pub' in column or 'ros' in column or 'mock' in column or 'monitor' in column:
                continue
            ax = data.plot(ax=ax, x='time', y=column, label=column)
            # plot_memory_usage(time, data[column], column)

    plt.ylabel('Memory usage in bytes')
    plt.xlabel('Time (s)')
    plt.title('Memory usage')
    # after plotting the data, format the labels
    current_values = plt.gca().get_yticks()
    plt.gca().set_yticklabels(['{:,.0f}'.format(x) for x in current_values])

    box = ax.get_position()
    ax.set_position([box.x0+box.width * 0.1, box.y0, box.width * 0.7, box.height])
    ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.grid()
    plt.show()


# data_columns = ['path_info_gain', 'path_cost', 'num_samples', 'tree_size']
data_columns = ['path_info_gain', 'num_samples', 'tree_size']
folder_path = '../test_results/2023-01-24_13:58:39-before/search_random_1' # Before
# folder_path2 = '../test_results/2023-01-27_17:26:03-hash/search_scenario_3' # After
# folder_path = '../test_results/2023-01-24_13:58:39-before/search_scenario_1' # Before
# folder_path2 = '../test_results/2023-01-27_17:26:03-hash/search_random_1' # After
# folder_path = '../test_results/2023-01-24_13:58:39-before/search_random_1' # Before
# folder_path2 = '../test_results/2023-01-27_17:26:03-hash/search_random_1' # After
# folder_path3 = '../test_results/2023-02-15_17:39:04-hash-int2/search_random_1' # After
# folder_path4 = '../test_results/2023-02-16_10:45:12-hash-int3/search_random_1' # After
folder_path5 = '../test_results/2023-02-16_12:01:14-hash-int4/search_random_1' # After
# folder_path6 = '../test_results/2023-02-24_15:07:10-no-recurse/search_random_1' # After
# folder_path7 = '../test_results/2023-02-24_17:44:20-no-recurse/search_random_1' # After
folder_path7 = '../test_results/2023-02-25_15:12:23-resurse/search_random_1' # After
folder_path8 = '../test_results/2023-02-27_09:00:41/search_random_1' # After 
folder_path9 = '../test_results/2023-03-08_10:59:26-extend-radius/search_random_1' # After 
for column_name in data_columns:
    # Plot the ...
    fig, ax = plt.subplots(figsize=(10,4))
    loop_over_files_and_plot(folder_path, 'red', ax, column_name)
    # loop_over_files_and_plot(folder_path2, 'orange', ax, column_name)
    # loop_over_files_and_plot(folder_path3, 'green', ax, column_name)
    # loop_over_files_and_plot(folder_path4, 'blue', ax, column_name)
    # loop_over_files_and_plot(folder_path5, 'purple', ax, column_name)
    # loop_over_files_and_plot(folder_path6, 'green', ax, column_name)
    # loop_over_files_and_plot(folder_path7, 'green', ax, column_name)
    loop_over_files_and_plot(folder_path8, 'orange', ax, column_name)
    loop_over_files_and_plot(folder_path9, 'blue', ax, column_name)

    plt.ylabel(column_name)
    plt.xlabel('Time (s)')
    plt.title(column_name)
    ax.get_legend().remove()
    plt.grid()
    plt.show()


mem_file1 = '../test_results/2023-01-24_13:58:39-before/search_scenario_3/cpu_mem_metrics.csv' # Before
mem_file1 = '../test_results/2023-01-27_17:26:03-hash/search_random_1/cpu_mem_metrics.csv' # After
mem_file2 = '../test_results/2023-02-15_17:39:04-hash-int2/search_random_1/cpu_mem_metrics.csv' # After
mem_file2 = '../test_results/2023-03-08_10:59:26-extend-radius/search_random_1/cpu_mem_metrics.csv' # After 
data = pd.read_csv(mem_file1, sep=',')
plot_mem_cpu(data, 'mem')
data = pd.read_csv(mem_file2, sep=',')
plot_mem_cpu(data, 'mem')




# Which results do I want to see? All of the metrics, mean, and standard deviation and lines for each run
# For all scenarios


