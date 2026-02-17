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


def process_data_func_for_executed(directory_paths):
    # Initialize a list to store the compiled results
    file_results = {}
    file_results_planner = {}

    for directory_path in directory_paths:
        # Iterate over each directory passed in
        for subdir in os.listdir(directory_path):
            subdir_path = os.path.join(directory_path, subdir)
            # Iterate through each param setup directory
            if os.path.isdir(subdir_path):
                # Iterate through each CSV file for each run
                for file in os.listdir(subdir_path):
                    file_path = os.path.join(subdir_path, file)
                    # Check if the file is a CSV file
                    if file.endswith('.csv') and file.startswith('tigris_search'):
                        # Read the CSV file into a pandas DataFrame
                        df = pd.read_csv(file_path)
                        
                        # Perform any necessary data processing or analysis
                        new_row = df.iloc[-1] # Add the last row
                        # Normalize the average_entropy and then add it back to new_row
                        normalize_val = np.max(df[' average_entropy'])
                        yval_from_df = 100-100*(new_row[' average_entropy']/normalize_val)
                        new_row['normalized_average_entropy'] = yval_from_df

                        # Check if the file name is in the dictionary
                        if subdir in file_results:
                            file_results[subdir].append(new_row)
                        else:
                            file_results[subdir] = [new_row]

                        # add the file name to the beginning of the new row
                        # file_results.append(new_row)
                        # Append the results to the list
                        # results.append(df)

                    if file.endswith('.csv') and file.startswith('tigris_pl'):
                        # Read the CSV file into a pandas DataFrame
                        df = pd.read_csv(file_path)
                        
                        # Perform any necessary data processing or analysis
                        # new_row = df.iloc[-1] # Add the last row
                        new_row = df.iloc[0] # Add the first row
                        # Check if the file name is in the dictionary
                        if subdir in file_results_planner:
                            file_results_planner[subdir].append(new_row)
                        else:
                            file_results_planner[subdir] = [new_row]

    results = np.empty((0, 10), dtype=np.float64)
    results_names = []
    for subdir in file_results:    
        # take the average of the results from each file
        # and append it to the results list

        # print subdir and number of files
        print(subdir, " ", len(file_results[subdir]))
        
        path_info_gain = [df['path_info_gain'] for df in file_results_planner[subdir]]
        average_path_info_gain = sum(path_info_gain) / len(path_info_gain)
        if len(file_results[subdir]) >30:
            hey1 = np.mean(path_info_gain[0:25])
            hey2 = np.mean(path_info_gain[25:50])

        path_cost = [df['path_cost'] for df in file_results_planner[subdir]]
        average_path_cost = sum(path_cost) / len(path_cost)

        num_waypoints = [df['num_waypoints'] for df in file_results_planner[subdir]]
        average_num_waypoints = sum(num_waypoints) / len(num_waypoints)

        num_nodes_in_path = [df['num_nodes_in_path'] for df in file_results_planner[subdir]]
        average_num_nodes_in_path = sum(num_nodes_in_path) / len(num_nodes_in_path)

        num_samples = [df['num_samples'] for df in file_results_planner[subdir]]
        average_num_samples = sum(num_samples) / len(num_samples)

        tree_size = [df['tree_size'] for df in file_results_planner[subdir]]
        average_tree_size = sum(tree_size) / len(tree_size)

        entropy = [df[' average_entropy'] for df in file_results[subdir]]
        average_average_entropy = sum(entropy) / len(entropy)
        ste_average_entropy = np.std(entropy) / np.sqrt(len(entropy))
        normalized_entropy = [df['normalized_average_entropy'] for df in file_results[subdir]]
        normalized_average_entropy = np.mean(normalized_entropy)
        ste_normalized_average_entropy = np.std(normalized_entropy) / np.sqrt(len(normalized_entropy))

        if len(file_results[subdir]) >30:
            hey1 = np.mean(entropy[0:25])
            hey2 = np.mean(entropy[25:50])

        results_names.append(subdir)
        new_row = np.array([
            average_average_entropy, 
            average_path_info_gain, 
            average_path_cost, 
            average_num_waypoints, 
            average_num_nodes_in_path, 
            average_num_samples, 
            average_tree_size,
            ste_average_entropy,
            normalized_average_entropy,
            ste_normalized_average_entropy
        ], dtype=np.float64)        
        results = np.append(results, [new_row], axis=0)
                    
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
    titles = ['Average Entropy', 
              'Average Path Info Gain', 
              'Average Path Cost', 
              'Average Num Waypoints', 
              'Average Num Nodes in Path', 
              'Average Num Samples', 
              'Average Tree Size',
              'STE Entropy',
              'Normalized Average Entropy',
              'STE Normalized Entropy']
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(titles[column_eval])

    max_val = max(results[:,column_eval])
    min_val = min(results[:,column_eval])

    # print the name for the min and max value
    min_index = np.argmin(results[:,column_eval])
    max_index = np.argmax(results[:,column_eval])
    print("Min: ", results_names[min_index])
    print("Max: ", results_names[max_index])

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
    plt.pause(0.1)

# Define the directory path
directory_path = "/home/moon/code/onr_ws/src/ipp_planners/test_results/param_sweep/"
directory_path_v2 = "/home/moon/code/onr_ws/src/ipp_planners/test_results/param_sweep_execute_25_v2/"
directory_path_tests1 = "/home/moon/code/onr_ws/src/ipp_planners/test_results/tests1/"
directory_paths = [directory_path_v2, directory_path_tests1]
# directory_paths = ["/home/moon/code/onr_ws/src/ipp_planners/test_results/param-sweep-v29/"]

process_data = True
if process_data:
    # process_data_func(directory_path)
    process_data_func_for_executed(directory_paths)

# Load results_names and results
with open('data_25.pkl', 'rb') as f:
    data = pickle.load(f)


results_names = data['results_names']
results = data['results']

# Plot the results
# plot_param_sweep_non_executed(results, results_names)
plot_param_sweep_executed(results, results_names)
# plot_param_sweep_executed(results, results_names, 2) # path cost
plot_param_sweep_executed(results, results_names, 7) # STE Entropy
plot_param_sweep_executed(results, results_names, 8) # Normalized Average Entropy
plot_param_sweep_executed(results, results_names, 9) # STE Normalized Entropy

