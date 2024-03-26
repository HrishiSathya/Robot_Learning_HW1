import os
import time

def main():
    traj = [] # insert here
    for state in traj:
        os.system("ros2 service call /xarm_pose_plan xarm_msgs/srv/PlanPose \"{target:{ position: {x: %s, y: %s, z: %s}, orientation: {x: %s, y: %s, z: %s, w: %s}}}\"" % (state[0], state[1], state[2], state[3], state[4], state[5], state[6]))
        os.system("ros2 service call /xarm_exec_plan xarm_msgs/srv/PlanExec \"{wait: true}\"")

if __name__ == "__main__":
    main()