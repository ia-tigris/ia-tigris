#!/usr/bin/env python
import os
import yaml
import pickle
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from matplotlib.collections import PatchCollection
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.gridspec as gridspec
import matplotlib.patches as patches

curr_dir = os.getcwd()
sr_path = curr_dir + "/../search_requests/"

def gen_random_search_map(width, height, num_gaussians, cov_range):
    #create empty search map
    search_map = np.zeros((height, width)) #matrix indexing
    #randomly sample means and generate covariance matrices
    means = np.random.randint([0,0], search_map.shape, (num_gaussians,2))
    std_devs = np.random.uniform(cov_range[0],cov_range[1],(num_gaussians,2))
    covariances = [np.array([[std_devs[k,0]**2, 0],[0, std_devs[k,1]**2]]) for k in range(len(std_devs))]
    #generate meshgrid with matrix indexing
    X,Y = np.meshgrid(np.arange(search_map.shape[0]), np.arange(search_map.shape[1]), indexing='ij')
    pos = np.dstack((X,Y))
    #generate gaussian distributions
    for idx in range(num_gaussians):
        gaussian_blob = multivariate_normal(means[idx],covariances[idx]).pdf(pos)
        search_map = np.sum((search_map, gaussian_blob),axis=0)
    return search_map

# taken from https://stackoverflow.com/questions/8090229/resize-with-averaging-or-rebin-a-numpy-2d-array
# taken from https://gist.github.com/zonca/1348792
def rebin(a, new_shape):
    M, N = a.shape
    m, n = new_shape
    #downsample if smaller size otherwise upsample
    if m < M and n < N:
        bin_shape = int(m),int(M//m),int(n),int(N//n)
        new_map = a.reshape(bin_shape).mean(-1).mean(1)
    else:
        new_map = np.repeat(np.repeat(a, m/M, axis=0), n/N, axis=1)
    return new_map / np.max(new_map)

#assume that discretization is a multiple of bounds
def generate_grid_priors(search_map, discretization):
    #normalize the search grid
    norm_search_map = search_map / np.max(search_map)
    #downsample based on discretization
    new_shape = tuple(int(val/discretization) for val in norm_search_map.shape)
    downsampled_search_map = rebin(search_map, new_shape)
    #upsample so it's the same size as the original search map
    upsampled_search_map = rebin(downsampled_search_map, search_map.shape)
    #determine x and y coordinates of search priors
    x_vals = np.arange(0, upsampled_search_map.shape[0]+discretization, discretization)
    y_vals = np.arange(0, upsampled_search_map.shape[1]+discretization, discretization)
    #generate search prior coordinates from x and y vals
    search_priors = []
    for xmin,xmax in zip(x_vals[:-1],x_vals[1:]):
        for ymin,ymax in zip(y_vals[:-1],y_vals[1:]):
            square_coords = [(ymin,xmin,0), (ymax,xmin,0),(ymax,xmax,0),(ymin,xmax,0)]
            search_priors.append(square_coords)
    return upsampled_search_map, downsampled_search_map, np.array(search_priors)

# weighted sample to determine poses of ships
def gen_weighted_sample(probs, num_samples):
    #upsample so we pick coordinates in original map
    probs_ravel = probs.ravel()/float(probs.sum())
    ravel_idx = np.random.choice(len(probs_ravel), num_samples, p=probs_ravel,replace=False)
    x,y = np.unravel_index(ravel_idx, probs.shape)
    return np.column_stack((y,x))

def generate_random_search_requests(params):
    for idx in range(params['num_search_requests']):
        rand_search_map = gen_random_search_map(params['map_width'],params['map_height'],
                                params['num_gaussians'],(params['min_std_dev'], params['max_std_dev']))
        upsampled_search_map, downsampled_search_map, grid_priors = generate_grid_priors(rand_search_map, 
                                                        params['prior_discretization'])
        target_poses = gen_weighted_sample(upsampled_search_map, params['num_samples'])
        #add grid prior, search map, and target poses to dictionary
        search_request = {'map_width': params['map_width'],
                          'map_height': params['map_height'],
                          'search_map': downsampled_search_map*0.5, #scale map for plan request from 0.0 --> 0.5
                          'grid_priors': grid_priors,
                          'target_poses': target_poses}
        #write file
        filename = "sr_%i_%i_%i" % (params['map_width'],params['map_height'],idx+1)
        with open(sr_path + filename + ".pickle", 'wb') as handle:
            pickle.dump(search_request, handle, protocol=pickle.HIGHEST_PROTOCOL) #change to protocol=2 if using python2.7

        print(str(idx+1) + " / " + str(params['num_search_requests']) + " complete")


        fig = plt.figure(figsize=(15, 10))
        gs = fig.add_gridspec(2, 2)
        #plot surface and contours
        ax1=fig.add_subplot(gs[0,:], projection='3d')
        X,Y = np.meshgrid(np.arange(0,params['map_width']), np.arange(0,params['map_height']))
        ax1.plot_surface(X, Y, (rand_search_map/np.max(rand_search_map))+1, 
                cmap='viridis', linewidth=1) #offset so it's above contour map
        ax1.contourf(X,Y,rand_search_map,cmap='viridis')
        #plot original search map
        ax2 = fig.add_subplot(gs[1,0])
        norm_search_map = rand_search_map / np.max(rand_search_map)
        sm = ax2.matshow(norm_search_map*0.5)
        fig.colorbar(sm, ax=ax2,shrink=0.7)
        #plot search priors and target locations
        ax3 = fig.add_subplot(gs[1,1])
        um = ax3.matshow(upsampled_search_map*0.5)
        lower_left_coords = grid_priors[:,3]
        upper_right_coords = grid_priors[:,1]
        rectangles = []
        for lower_left,upper_right in zip(lower_left_coords, upper_right_coords):
            rect = patches.Rectangle((lower_left[0],lower_left[1]),
                                    upper_right[0] - lower_left[0],
                                    upper_right[1] - lower_left[1],
                                    fill = False) #cartesian
            rectangles.append(rect)
        ax3.add_collection(PatchCollection(rectangles, match_original=True))
        fig.colorbar(um, ax=ax3,shrink=0.7)
        ax3.set_xlim(0,params['map_width'])
        ax3.set_ylim(0,params['map_height'])
        plt.scatter(target_poses[:,0], target_poses[:,1], color='red') #cartesian
        fig.savefig(sr_path + filename + ".png")
        if params['visualize']:
            plt.show()

    

if __name__ == "__main__":
    #create write directory if it doesn't exist
    os.makedirs(os.path.dirname(sr_path), exist_ok=True)
    #load parameters
    with open(curr_dir + "/../config/gen_search_map.yaml", "r") as stream:
        params = yaml.safe_load(stream)
    #generate search requests
    generate_random_search_requests(params)

