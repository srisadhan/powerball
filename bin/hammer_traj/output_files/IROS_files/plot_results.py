import numpy
import pandas as pd
from pathlib import Path
import datetime
import numpy as np
import sys

import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from matplotlib.ticker import FormatStrFormatter
import seaborn as sns

# sns.set_style("whitegrid")
plot_magnet     = True

if plot_magnet: 
    # set different parameters for the new plot
    params = {'legend.fontsize': 20,
            #   'figure.figsize': (10, 5),
            'axes.labelsize': 20,
            #  'axes.titlesize':'x-large',
            'xtick.labelsize':16,
            'ytick.labelsize':16}

    pylab.rcParams.update(params)
else:
    params = {'legend.fontsize': 14,
            #   'figure.figsize': (10, 5),
            'axes.labelsize': 16,
            #  'axes.titlesize':'x-large',
            'xtick.labelsize':16,
            'ytick.labelsize':16}
    pylab.rcParams.update(params)

# parameters
freq            = 200
mov_avg_wnd     = 4
N_subplots      = 2
ind             = -200 # extra data collected after the experiment

stiff_config    = ['high_stiffness', 'low_stiffness', 'variable_stiffness']
label           = {'robot' : 'robot', 'low_stiffness': 'LS', 'variable_stiffness': 'VS', 'high_stiffness': 'HS'}
runs            = ['run1', 'run2', 'run3', 'run4', 'run5']
exps            = ['optim_tf', 'tf_03', 'tf_20']

plot_style      = {'robot' : '-.', 'low_stiffness': '-', 'variable_stiffness': '-', 'magnet': '-', 'high_stiffness':'-'}
plot_color      = {'robot' : 'k',  'low_stiffness': 'b', 'variable_stiffness': 'r', 'magnet': 'k', 'high_stiffness':'k'}


def parse_data_from_csv(runs, exp, setting):
    hammer_disp = pd.DataFrame()
    hammer_vel  = pd.DataFrame()
    robot_disp  = pd.DataFrame()
    robot_vel   = pd.DataFrame()
    time        = pd.DataFrame()
    
    for run in runs:

        # load the files
        filepath = str(Path(__file__).parents[0] / run / exp / setting) + "_output.csv"
        data     = pd.read_csv(filepath, delimiter=',')
        
        if exp != 'optim_tf':
            filepath_opti = str(Path(__file__).parents[0] / 'traj_opti' / exp) + "/variable_stiffness.txt"
            magnet_pos    = np.genfromtxt(filepath_opti, 
                                        dtype=float, 
                                        delimiter='', 
                                        usecols=6, 
                                        unpack=True, 
                                        encoding=None)
        else:
            magnet_pos    = np.empty((0,0))

        tvec     = np.zeros((data.shape[0], 1))
        counter  = 0

        for t in data['Time']:
            temp = t.split(':')

            tvec[counter,0] = int(temp[0]) * 3600 + int(temp[1]) * 60 + int(temp[2]) + int(temp[3])/1e6
            counter += 1

        tvec = tvec - tvec[0,0]
        bd   = np.array(-data['Y']).reshape((data.shape[0], 1))
        bv   = np.multiply(np.diff(bd, axis=0), freq).reshape((data.shape[0]-1, 1))
        hd   = np.array(-data['Y'] + data['handle_disp']).reshape((data.shape[0], 1))
        hv   = np.multiply(np.diff(hd, axis=0), freq).reshape((data.shape[0]-1, 1))
        
        if hammer_disp.empty:
            time        = pd.DataFrame({'time' : tvec[1:ind,0]})
            robot_disp  = pd.DataFrame({run : bd[1:ind,0]})
            robot_vel   = pd.DataFrame({run : bv[ :ind,0]})

            hammer_disp = pd.DataFrame({run : hd[1:ind,0]})                
            hammer_vel  = pd.DataFrame({run : hv[ :ind,0]})
            
        else:
            hammer_disp = pd.concat([hammer_disp, pd.DataFrame({run : hd[1:ind,0]})], axis=1)
            hammer_vel  = pd.concat([hammer_vel,  pd.DataFrame({run : hv[ :ind,0]})], axis=1)
            robot_disp  = pd.concat([robot_disp,  pd.DataFrame({run : bd[1:ind,0]})], axis=1)
            robot_vel   = pd.concat([robot_vel,   pd.DataFrame({run : bv[ :ind,0]})], axis=1)

    return time, hammer_disp, hammer_vel, robot_disp, robot_vel, magnet_pos


for exp in exps:
    
    if plot_magnet is False:
        fig, ax = plt.subplots(N_subplots,1)

        if exp is 'tf_03' or exp is 'tf_05':
            ylim      = [-0.2, 0.1]
            ax[0].set_ylabel('Displacement (m)')
            ax[1].set_ylabel('Velocity (m/s)')    
        else:
            ylim      = [-0.2, 0.1]
    
    linewidth = 1.5

    tf = 0
    for setting in stiff_config:

        if plot_magnet and exp == 'tf_20':
            mag_fig, mag_ax = plt.subplots()

        time, hammer_disp, hammer_vel, robot_disp, robot_vel, magnet_pos = parse_data_from_csv(runs, exp, setting)

        # apply moving average to the hammer velocity
        for j in runs:
            hammer_vel[j] = hammer_vel[j].rolling(mov_avg_wnd, center=True).mean()
            
        tf =  max(tf, time['time'].iloc[-1])
        
        if plot_magnet is False:

            # displacement plots
            ax[0].fill_between(time['time'], hammer_disp.min(axis=1), hammer_disp.max(axis=1),color=plot_color[setting], alpha=0.25)

            ax[0].plot(time['time'], hammer_disp.mean(axis=1), color=plot_color[setting], label=label[setting], linestyle=plot_style[setting])
            ax[0].plot(time['time'], 0.05 + 0 * time['time'], color='k', linestyle=':')

            # velocity plots
            if setting == 'high_stiffness':
                ax[1].fill_between(time['time'], robot_vel.min(axis=1), robot_vel.max(axis=1),color=plot_color[setting], alpha=0.25)
            else:
                ax[1].fill_between(time['time'], hammer_vel.min(axis=1), hammer_vel.max(axis=1),color=plot_color[setting], alpha=0.25)

            ax[1].plot(time['time'], hammer_vel.mean(axis=1), color=plot_color[setting], linestyle=plot_style[setting])

            if exp == 'optim_tf':
                ax[0].plot([0.81 * time['time'].iloc[-1], 0.81 * time['time'].iloc[-1]], [-0.2, 0.1], color='k', linestyle=plot_style['robot'])
                ax[1].plot([0.81 * time['time'].iloc[-1], 0.81 * time['time'].iloc[-1]], [-1, 1.25], color='k', linestyle=plot_style['robot'])
        elif exp == 'tf_20':
            if setting == 'variable_stiffness':
                mag_ax.plot(time['time'], ( magnet_pos - 0.03) * 1000,color=plot_color['magnet'], linestyle=plot_style['magnet'], label='magnets', linewidth=2)
                mag_ax.plot(time['time'], (-magnet_pos + 0.03) * 1000,color=plot_color['magnet'], linestyle=plot_style['magnet'], linewidth=2)
                mag_ax.plot(time['time'], (hammer_disp.mean(axis=1) - robot_disp.mean(axis=1)) * 1000,color=plot_color[setting], linestyle=plot_style[setting], label='VS',linewidth=2)
                mag_ax.tick_params(axis='y',        # changes apply to the y-axis
                        which='both',      # both major and minor ticks are affected
                        left=False,      # ticks along the bottom edge are off
                        right=False,         # ticks along the top edge are off
                        labelleft=False) # labels along the bottom edge are off

            else:
                mag_ax.plot(time['time'], 0 * time['time'] + ( 0.03) * 1000,color=plot_color['magnet'], linestyle=plot_style['magnet'], label='magnets',linewidth=2)
                mag_ax.plot(time['time'], 0 * time['time'] + (-0.03) * 1000,color=plot_color['magnet'], linestyle=plot_style['magnet'],linewidth=2)
                mag_ax.plot(time['time'], (hammer_disp.mean(axis=1) - robot_disp.mean(axis=1)) * 1000,color=plot_color[setting], linestyle=plot_style[setting], label='LS',linewidth=2)
                mag_ax.set_ylabel('Displacement (mm)')

            # mag_ax.grid()
            mag_ax.set_xlim([0, tf])
            mag_ax.set_xticks(np.arange(0, 2.0 + 0.5, step=0.5))
            mag_ax.set_ylim([-.035 * 1000, .035 * 1000])
            mag_ax.set_xlabel('Time (s)') 
            mag_ax.legend(ncol=2, loc='upper center', frameon=False, bbox_to_anchor=(0.5, 1.175))
            plt.tight_layout()

    if plot_magnet is False:

        ax[0].set_xlim([0, tf])
        ax[1].set_xlim([0, tf])

        ax[0].set_ylim(ylim)            
        ax[0].yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
        
        if exp == 'tf_03':
            ax[0].legend(loc='lower left')
        else:
            ax[0].legend(loc='lower right')
        
        
        ax[1].set_ylim([-1, 1.25])  
        ax[1].yaxis.set_major_formatter(FormatStrFormatter('%.1f')) 
        ax[1].set_xlabel('Time (s)')

        if exp == 'tf_20':
            ax[0].plot([0.81 * tf, 0.81 * tf], [-0.2, 0.1], color='k', linestyle=plot_style['robot'])
            ax[1].plot([0.81 * tf, 0.81 * tf], [-1, 1.25],  color='k', linestyle=plot_style['robot'])

            ax[0].set_xticks(np.arange(0,2.0 + 0.5, step = 0.5))
            ax[1].set_xticks(np.arange(0,2.0 + 0.5, step = 0.5))
        elif exp == 'tf_03':
            ax[0].plot([0.85 * tf, 0.85 * tf], [-0.2, 0.1], color='k', linestyle=plot_style['robot'])
            ax[1].plot([0.85 * tf, 0.85 * tf], [-1, 1.25],  color='k', linestyle=plot_style['robot'])

            ax[0].set_xticks(np.arange(0,0.3 + 0.1, step = 0.1))
            ax[1].set_xticks(np.arange(0,0.3 + 0.1, step = 0.1))
        else:
            ax[0].set_xticks(np.arange(0,2.0, step = 0.3))
            ax[1].set_xticks(np.arange(0,2.0, step = 0.3))

        ax[0].tick_params(axis='x',        # changes apply to the x-axis

                        which='both',      # both major and minor ticks are affected
                        bottom=False,      # ticks along the bottom edge are off
                        top=False,         # ticks along the top edge are off
                        labelbottom=False) # labels along the bottom edge are off
        if exp != 'tf_03':
            for j in range(0,2):
                ax[j].tick_params(axis='y',        # changes apply to the x-axis
                            which='both',      # both major and minor ticks are affected
                            left=False,      # ticks along the left edge are off
                            right=False,         # ticks along the right edge are off
                            labelleft=False) # labels along the left edge are off
            

        for i in range(0,N_subplots):
            # ax[i].grid(b=True, which='minor', axis='both')
            ax[i].grid()
            ax[i].yaxis.set_major_formatter(FormatStrFormatter('%.1f')) 
   

    plt.tight_layout()
    plt.subplots_adjust(hspace=0.1)

# plt.show() 

# Force plots 
stiff_config    = ['high_stiffness', 'low_stiffness', 'variable_stiffness']
ind             = -1

force_fig, force_ax = plt.subplots()
for setting in stiff_config:
    # load the files
    filepath = str(Path(__file__).parents[0] / 'force/tf_20' / setting) + "_output.csv"
    data     = pd.read_csv(filepath, delimiter=',')

    tvec     = np.zeros((data.shape[0], 1))
    counter  = 0

    for t in data['Time']:
        temp = t.split(':')

        tvec[counter,0] = int(temp[0]) * 3600 + int(temp[1]) * 60 + int(temp[2]) + int(temp[3])/1e6
        counter += 1

    tvec = tvec - tvec[0,0]
    Fy   = np.array(data['Fy']).reshape((data.shape[0], 1))
    Mx   = np.array(data['Mx']).reshape((data.shape[0], 1))

    # remove the extra data collected for the last 1 sec
    # tvec    = tvec[:ind,0]
    # Fy      = Fy[:ind,0]
    # Mx      = Mx[:ind,0]

    force_ax.plot(tvec, Fy, linestyle=plot_style[setting], color=plot_color[setting], label=label[setting])
    force_ax.fill_betweenx([-30, 15], 0.8*tvec[-1], tvec[-1], color=(169/255, 169/255, 169/255, 0.1))

    force_ax.set_xlabel('Time (s)')
    force_ax.set_ylabel('Force (N)')
    force_ax.set_xlim([0, tvec[-1]])
    force_ax.set_ylim([-30, 15])
    plt.grid()
    plt.legend(loc='lower left')
    plt.tight_layout()

plt.show()