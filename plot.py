from matplotlib import pyplot as plt
import numpy as np

# Initialize number of trials
trials = 3

def plot(loss_list, colors):

    # Create a box and whisker plot
    plt.boxplot(loss_list, vert=True, patch_artist=True)

    # Add labels and title
    plt.xlabel('Trial (seeds 10, 40, 80 respectively)', fontweight='bold', fontsize=13)
    plt.ylabel('loss value', fontweight='bold', fontsize=9)
    plt.xlim(-0.1,trials+0.5)
    plt.title('Loss during Sophie Kitchen Experiments', fontweight='bold', fontsize=13)

    # Customize colors
    for patch, color in zip(plt.boxplot(loss_list, vert=True, patch_artist=True)['boxes'], colors):
        patch.set_facecolor(color)

    # Show the plot
    plt.yscale('log')

# load files
random_datalog = np.loadtxt('totalSophieloss.txt', delimiter=',')

# set colors for each box
col = []
for i in range(trials):
    col.append('lightgreen')

plot(random_datalog, col) 

"""
Plot
"""
states_legends_handle = [plt.plot([0],[0], color="green", label="Sophie loss per trial")] #linestyle="solid"
plt.legend(loc="upper right") 
fig_size = (20, 10)
plt.gcf().set_size_inches(fig_size)
plt.savefig("Sophieloss.png")
plt.show()