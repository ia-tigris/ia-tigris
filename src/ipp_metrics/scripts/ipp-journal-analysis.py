import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import pickle
from matplotlib import cm
from matplotlib import colors
from matplotlib.animation import FuncAnimation
from scipy.io import savemat

# Use env


def process_data_func(directory_path):
    # Initialize a list to store the compiled results
    results = np.empty((0, 6), dtype=np.float64)
    results_names = []

    # Iterate through each directory in the main directory
    for subdir in os.listdir(directory_path):
        subdir_path = os.path.join(directory_path, subdir)
        
        # Check if the subdirectory is a directory
        if os.path.isdir(subdir_path):
            # Iterate through each CSV file in the subdirectory
            file_results = []
            for file in os.listdir(subdir_path):
                file_path = os.path.join(subdir_path, file)
                # Check if the file is a CSV file
                if file.endswith('.csv') and file.startswith('tigris_pl'):
                    # Read the CSV file into a pandas DataFrame
                    df = pd.read_csv(file_path)
                    
                    # Perform any necessary data processing or analysis
                    new_row = df.iloc[-1]
                    # add the file name to the beginning of the new row
                    file_results.append(new_row)
                    # Append the results to the list
                    # results.append(df)
            # take the average of the results from each file
            # and append it to the results list
            path_info_gain = [df['path_info_gain'] for df in file_results]
            average_path_info_gain = sum(path_info_gain) / len(path_info_gain)

            path_cost = [df['path_cost'] for df in file_results]
            average_path_cost = sum(path_cost) / len(path_cost)

            num_waypoints = [df['num_waypoints'] for df in file_results]
            average_num_waypoints = sum(num_waypoints) / len(num_waypoints)

            num_nodes_in_path = [df['num_nodes_in_path'] for df in file_results]
            average_num_nodes_in_path = sum(num_nodes_in_path) / len(num_nodes_in_path)

            num_samples = [df['num_samples'] for df in file_results]
            average_num_samples = sum(num_samples) / len(num_samples)

            tree_size = [df['tree_size'] for df in file_results]
            average_tree_size = sum(tree_size) / len(tree_size)

            results_names.append(subdir)
            new_row = np.array([average_path_info_gain, average_path_cost, average_num_waypoints, average_num_nodes_in_path, average_num_samples, average_tree_size], dtype=np.float64)
            results = np.append(results, [new_row], axis=0)
                    
    # Save results_names and results
    data = {'results_names': results_names, 'results': results}
    savemat('data_25.mat', data)
    with open('data.pkl', 'wb') as f:
        pickle.dump(data, f)

def save_data_append(file_path, xvals, yvals, ax, line_color):
    df = pd.read_csv(file_path)
    time_offset = df['time'][0]
    normalize_val = df[' average_entropy'][0]
    yval_from_df = 100-100*(df[' average_entropy']/normalize_val)
    # yval_from_df = df[' average_entropy']
    yinterp = np.interp(xvals, df['time']-time_offset, yval_from_df)
    yvals.append(yinterp)
    # ax.plot(df['time']-time_offset, yval_from_df, color=line_color, linewidth=1, alpha=0.2)

def process_data_func_for_executed(directory_paths):
    # Initialize a list to store the compiled results
    results = np.empty((0, 7), dtype=np.float64)
    xvals = np.linspace(0, 320, 1000)
    yvals_tigris  = []
    yvals_greedy  = []
    yvals_mcts  = []
    yvals_random  = []

    fig = plt.figure()
    ax = fig.add_subplot(111)
    
    for directory_path in directory_paths: # iterate directories
        for subdir in os.listdir(directory_path): # iterate over jobs
            subdir_path = os.path.join(directory_path, subdir)
            
            # Check if the subdirectory is a directory
            if os.path.isdir(subdir_path):
                # Iterate through each CSV file in the subdirectory
                for file in os.listdir(subdir_path):
                    file_path = os.path.join(subdir_path, file)
                    # Check if the file is a CSV file
                    if file.endswith('.csv') and file.startswith('tigris_search'):
                        save_data_append(file_path, xvals, yvals_tigris, ax, 'r')
                    elif file.endswith('.csv') and file.startswith('greedy_search'):
                        save_data_append(file_path, xvals, yvals_greedy, ax, 'g')
                    elif file.endswith('.csv') and file.startswith('mcts_search_search'):
                        save_data_append(file_path, xvals, yvals_mcts, ax, 'k')
                    elif file.endswith('.csv') and file.startswith('random_search'):
                        save_data_append(file_path, xvals, yvals_random, ax, 'b')

    yvals_tigris = np.array(yvals_tigris)
    yvals_greedy = np.array(yvals_greedy)
    yvals_mcts = np.array(yvals_mcts)
    yvals_random = np.array(yvals_random)

    # Calculate the standard deviation
    sem_tigris = np.std(yvals_tigris, axis=0)
    sem_greedy = np.std(yvals_greedy, axis=0)
    sem_random = np.std(yvals_random, axis=0)
    sem_mcts = np.std(yvals_mcts, axis=0)

    # Calculate the 95% confidence interval
    ci_tigris = sem_tigris * 1.96/np.sqrt(yvals_tigris.shape[0])
    ci_greedy = sem_greedy * 1.96/np.sqrt(yvals_greedy.shape[0])
    ci_random = sem_random * 1.96/np.sqrt(yvals_random.shape[0])
    ci_mcts = sem_mcts * 1.96/np.sqrt(yvals_mcts.shape[0])

    # Plot the average and the 95% confidence interval
    ax.plot(xvals, np.mean(yvals_tigris, axis=0), color='r', label='Average Tigris', linewidth=3)
    ax.fill_between(xvals, np.mean(yvals_tigris, axis=0) - ci_tigris, np.mean(yvals_tigris, axis=0) + ci_tigris, color='r', alpha=0.2)
    ax.plot(xvals, np.mean(yvals_greedy, axis=0), color='g', label='Average Greedy', linewidth=3)
    ax.fill_between(xvals, np.mean(yvals_greedy, axis=0) - ci_greedy, np.mean(yvals_greedy, axis=0) + ci_greedy, color='g', alpha=0.2)
    ax.plot(xvals, np.mean(yvals_random, axis=0), color='b', label='Average Random', linewidth=3)
    ax.fill_between(xvals, np.mean(yvals_random, axis=0) - ci_random, np.mean(yvals_random, axis=0) + ci_random, color='b', alpha=0.2)
    ax.plot(xvals, np.mean(yvals_mcts, axis=0), color='k', label='Average MCTS', linewidth=3)
    ax.fill_between(xvals, np.mean(yvals_mcts, axis=0) - ci_mcts, np.mean(yvals_mcts, axis=0) + ci_mcts, color='k', alpha=0.2)
    plt.legend()
    plt.show()
    # Save results_names and results
    data = {'results_names': results_names, 'results': results}
    savemat('data_25.mat', data)
    with open('data_25.pkl', 'wb') as f:
        pickle.dump(data, f)
    
    

def plot_param_sweep_non_executed(results, results_names, column_eval=0):
    # Create a figure and 3D axes for the graph
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    max_val = max(results[:,column_eval])
    min_val = min(results[:,column_eval])

    x_vals = []
    y_vals = []
    z_vals = []
    color_values = []
    sizes = []
    # Loop through each item in results_names and results
    for name, result in zip(results_names, results):
        # Parse the numbers from the name
        x = int(name.split('_')[1])
        y = int(name.split('_')[3])
        z = int(name.split('_')[5])

        # Use the row value of the results vector to determine the color and size of the spot
        color_value = (result[column_eval]-min_val) / (max_val-min_val)  # normalize the result to get a color value between 0 and 1
        size = (result[column_eval]-min_val) / (max_val-min_val) * 80 + 2  # adjust this factor to get a suitable size for the spots
        
        # Append the values to the lists
        x_vals.append(x)
        y_vals.append(y)
        z_vals.append(z)
        color_values.append(color_value)
        sizes.append(size)

    # Create a colormap
    norm = colors.Normalize(vmin=min_val, vmax=max_val)
    cmap = cm.get_cmap('jet')

    # Create a ScalarMappable object with the same normalization and colormap
    sm = cm.ScalarMappable(norm=norm, cmap=cmap)

    # Plot the scatter plot
    sc = ax.scatter(x_vals, y_vals, z_vals, c=color_values, s=sizes, cmap=cmap)

    # Add a colorbar
    plt.colorbar(sm)

    # Add labels to the graph
    ax.set_xlabel('Extend')
    ax.set_ylabel('Radius')
    ax.set_zlabel('Prune')

    # # Function to update the plot for each angle
    # def update(i):
    #     ax.view_init(elev=20., azim=i)

    # # Create an animation
    # ani = FuncAnimation(fig, update, frames=range(0, 360, 2), interval=100)

    # # Save the animation as a GIF
    # ani.save('3dscatter.gif', writer='imagemagick')

    # Show the graph
    plt.show()

def plot_param_sweep_executed(results, results_names, column_eval=0):
    # Create a figure and 3D axes for the graph
    titles = ['Average Entropy', 'Average Path Info Gain', 'Average Path Cost', 'Average Num Waypoints', 'Average Num Nodes in Path', 'Average Num Samples', 'Average Tree Size']
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(titles[column_eval])

    max_val = max(results[:,column_eval])
    min_val = min(results[:,column_eval])

    x_vals = []
    y_vals = []
    z_vals = []
    color_values = []
    sizes = []
    # Loop through each item in results_names and results
    for name, result in zip(results_names, results):
        # Parse the numbers from the name
        x = int(name.split('_')[1])
        y = int(name.split('_')[3])
        z = int(name.split('_')[5])

        # Use the row value of the results vector to determine the color and size of the spot
        color_value = (result[column_eval]-min_val) / (max_val-min_val)  # normalize the result to get a color value between 0 and 1
        size = (result[column_eval]-min_val) / (max_val-min_val) * 80 + 2  # adjust this factor to get a suitable size for the spots
        
        # Append the values to the lists
        x_vals.append(x)
        y_vals.append(y)
        z_vals.append(z)
        color_values.append(color_value)
        sizes.append(size)

    # Create a colormap
    norm = colors.Normalize(vmin=min_val, vmax=max_val)
    cmap = cm.get_cmap('jet')

    # Create a ScalarMappable object with the same normalization and colormap
    sm = cm.ScalarMappable(norm=norm, cmap=cmap)

    # Plot the scatter plot
    sc = ax.scatter(x_vals, y_vals, z_vals, c=color_values, s=sizes, cmap=cmap)

    # Add a colorbar
    plt.colorbar(sm, ax=ax)

    # Add labels to the graph
    ax.set_xlabel('Extend')
    ax.set_ylabel('Radius')
    ax.set_zlabel('Prune')

    # # Function to update the plot for each angle
    # def update(i):
    #     ax.view_init(elev=20., azim=i)

    # # Create an animation
    # ani = FuncAnimation(fig, update, frames=range(0, 360, 2), interval=100)

    # # Save the animation as a GIF
    # ani.save('3dscatter.gif', writer='imagemagick')

    # Show the graph
    plt.show()
    plt.pause(0.001)

# Define the directory path
# directory_path = "/home/moon/code/onr_ws/src/ipp_planners/test_results/param_sweep/"
# directory_path_v2 = "/home/moon/code/onr_ws/src/ipp_planners/test_results/param_sweep_execute_25_v2/"
# directory_path_tests1 = "/home/moon/code/onr_ws/src/ipp_planners/test_results/tests1/"
# directory_paths = [directory_path_v2, directory_path_tests1]

directory_paths = ["/home/moon/code/onr_ws/src/ipp_planners/test_results/ipp-journal-full"]
directory_paths = ["/home/moon/code/onr_ws/src/ipp_planners/test_results/ipp-journal-full-more-rand-2"]

process_data = True
if process_data:
    process_data_func_for_executed(directory_paths)

# Load results_names and results
with open('data_25.pkl', 'rb') as f:
    data = pickle.load(f)


results_names = data['results_names']
results = data['results']

# Plot the results
# plot_param_sweep_non_executed(results, results_names)
plot_param_sweep_executed(results, results_names)
plot_param_sweep_executed(results, results_names, 2)

