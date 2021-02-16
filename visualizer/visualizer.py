# -*- coding: utf-8 -*-
"""
Created on Mon Feb 15 18:00:33 2021

@author: Thiyagarajan Masilamani
Visualize the debug output of Particle filter
"""
import csv
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np
from tqdm import tqdm
import argparse

      
map_file = '../data/map_data.txt'
result_file = '../images/'
debug_data_init = '../debug/parameter-init.txt'
debug_data_update = '../debug/parameter-after-updateweights.txt'
debug_data_resample = '../debug/parameter-after-resample.txt'
debug_data_best_particle = '../debug/parameter-best-particle.txt'

num_particles = 100
total_time_steps = 15
        
def plot_map_data():
    # Read map data and draw the obstacles in red
    map_data =  list(csv.reader(open(map_file),delimiter="\t"))
    map_data = np.array(map_data)
    obs_map_x = (map_data[:,0]).astype(np.float)
    obs_map_y = (map_data[:,1]).astype(np.float) 
    
    best_particle_data =  list(csv.reader(open(debug_data_best_particle)))
    best_particle_data = np.array(best_particle_data)
    best_particle_data_x = (best_particle_data[:,0]).astype(np.float)
    best_particle_data_y = (best_particle_data[:,1]).astype(np.float)
    
    fig = plt.figure(figsize=(28, 16))
    plt.rc('xtick', labelsize=20)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=20)    # fontsize of the tick labels
    ax1 = fig.add_subplot(111)
    ax1.scatter(obs_map_x,obs_map_y, s=100, c='r', marker="s")
    ax1.scatter(best_particle_data_x,best_particle_data_y, s=2, c='b', marker="o")
    plt.savefig(result_file + 'graphic_track.jpg')

    print ('Map image stored at ' + result_file + 'graphic_track.jpg' )
    
def plot_particles_stage2():
    plt.rc('xtick', labelsize=20)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=20)    # fontsize of the tick labels
    # plot the points for all the time stamps
    update_data =  list(csv.reader(open(debug_data_update)))
    update_data = np.array(update_data)
    
    resample_data =  list(csv.reader(open(debug_data_resample)))
    resample_data = np.array(resample_data)
    
    best_particle_data =  list(csv.reader(open(debug_data_best_particle)))
    best_particle_data = np.array(best_particle_data).astype(np.float)
    for i in tqdm(range(total_time_steps)):
        #print ('{:d} ... {:d}'.format(i*num_particles, (i+1)*num_particles-1))
        ## fetch 100 items from the array from the 'updateWeights' array
        x_update = (update_data[(i*num_particles):((i+1)*num_particles),1]).astype(np.float)
        y_update = (update_data[(i*num_particles):((i+1)*num_particles),2]).astype(np.float) 
        weight_update = (update_data[(i*num_particles):((i+1)*num_particles),4]).astype(np.float)
        
        ## fetch 100 items from the array from the 'reSampling' array
        x_resample = (resample_data[(i*num_particles):((i+1)*num_particles),1]).astype(np.float)
        y_resample = (resample_data[(i*num_particles):((i+1)*num_particles),2]).astype(np.float) 
        weight_resample = (resample_data[(i*num_particles):((i+1)*num_particles),4]).astype(np.float)
        
        #prepare the color map
        cmap = mcolors.ListedColormap(plt.rcParams['axes.prop_cycle'].by_key()['color'])
        #with plt.style.context(('fivethirtyeight')):
        #plot the scatter charts - 1 - shows the particles after weights update
        fig, axes = plt.subplots(nrows=1, ncols=2, sharex=True, sharey=True,figsize=(28, 12))
              
        plt.subplots_adjust(wspace=0.01, hspace=0)
        
        im_update = axes[0].scatter(x_update, y_update, s=100, c=weight_update, cmap=cmap,  marker="o")
        axes[0].scatter(best_particle_data[i][0], best_particle_data[i][1], s=500, c='g', marker="*")
        axes[0].set_title('@Time step :{:d} - Particle after UpdateWeights'.format(i), fontsize=24)
        
        #plot the scatter charts - 2 - shows the particles after resanpling
        axes[1].scatter(x_resample, y_resample, s=100, c=weight_resample, cmap=cmap, marker="o")
        axes[1].set_title('@ Time step :{:d} - Particle after Resampling'.format(i), fontsize=24)
        
        fig.colorbar(im_update, pad=0.005)# , ax=ax_update)
        #fig.colorbar(im_resample, ax=ax_resample)
        for ax in axes:                
            ax.grid(b=True, which='major', color='#666666', linestyle='-')        
            ax.minorticks_on()
            ax.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
                     
        plt.savefig(result_file + 'plot_at_time_' + str(i) + '.jpg')
        plt.tight_layout()
        plt.close(fig)

    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualize the output from the Particle filter algorithm')
    parser.add_argument('--show_map_observation', action='store_true', dest='show_map_observation' , default=False)
    parser.add_argument('--show_particle_updates', type=int,  dest='show_particle_updates' , default=5)

    args =  parser.parse_args()
    total_time_steps = args.show_particle_updates
    
    if args.show_map_observation:
        plot_map_data()
        
    if args.show_particle_updates:
        plot_particles_stage2()
        print ('Results stored at folder (../images/plot_at_time_*.jpg)' )
    
