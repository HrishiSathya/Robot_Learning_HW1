# Overview

This is an implementation of both Sophie's Kitchen (SophiesKitchen_implement.py) and Tamer (tamer_implement.py) algorithms. In the respective folders you will find the code that contains both the training and execution phase. Each algorithm will save the Qtable in Qtable.npy. The learning rate and discount factor are held constant and the same for both algorithms. In the execution phase, a trajectory will be given based on the demonstration inputs by the user.

One important note is that the Q table is quite large, as it is of the size of the continuous state space (which we discretize over cubic regions of the space), and discretized action space of 8. The algorithms will update the elements of the q table using data that we have post processed from joint space. We do this post processing by making text files that contain end effector position, distance to the goal, as well as a scalar value of whether the distance to the goal is reduced.

We also define our loss function as a distance function between the end effector and the goal state, where we seek to minimize this loss.

## Tamer
The Tamer algorithm is trained using Q-value iteration. Then, during the execution phase, the user will input 1 or -1 as a reward to give, so that the robot end effector will minimize the cost function.

## Sophie's Kitchen
The Sophie's Kitchen algorithm is trained the same way as Tamer. Then, during the execution phase, the user will input a distance reward based on feedback from the robots  end effector distance to the goal.

## Plotting
plot.py will plot the box and whisker plots for every trial with a new seed for each algorithm. This is based on the loss output from each of the data, which is then stored in a text file, toalSophieloss.txt and totalTamerloss.txt, respectively. The plots are also shown. We show that the results are not ideal, as more data is needed to train such a big Q table, which led to the Q table being very sparse.
