import matplotlib.pyplot as plt
import datetime
import numpy as np

### Visualization ###
def visualizeActions(actions, images):
    # plot layout
    ACTION_DIM_LABELS = ['x','y','z','yaw','pitch','roll','grasp']
    
    # image strip:
    img_strip = np.concatenate(np.array(images[::3]), axis=1)
    
    figure_layout = [
        ['image']*len(ACTION_DIM_LABELS),
        ACTION_DIM_LABELS
    ]
    plt.rcParams.update({'font.size':12})
    fig, ax = plt.subplot_mosaic(figure_layout)
    fig.set_size_inches([45, 10])
    
    # plotting actions
    figActions = np.array(actions).squeeze()
    for action_dim, action_label in enumerate(ACTION_DIM_LABELS):
        # actions: batch, horizon, dim --> only plot action 1
        ax[action_label].plot(figActions[:, 0, action_dim], label='executed action')
        ax[action_label].set_title(action_label)
        ax[action_label].set_xlabel('Time in one episode')
        
    # plotting image observation strip
    ax['image'].imshow(img_strip)
    ax['image'].set_xlabel('Time in one episode (subsampled)')
    
    plt.legend()
    
    current_time = datetime.datetime.now()
    # curr_date = datetime.today().date()
    
    # save figure; name corresponds to current time and date
    # all figures are under output_figs directory
    plt.savefig(f'output_figs/{current_time.year}-{current_time.month}-{current_time.day}_{current_time.hour}-{current_time.minute}-{current_time.second}_actionTraj.png', bbox_inches='tight')