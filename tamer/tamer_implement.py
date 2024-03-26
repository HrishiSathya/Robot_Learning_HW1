import numpy as np
import math


class TAMER():

    def __init__(self) -> None:

        """
        Initialize:
            Q matrix: size of state space and action space
            Discount factor
            Learning Rate
            State Variable
            Initial Action
            Initial Sampling of possible envs from data
            Initial Sampling of possible goals from data
            Initial sampling of trajectories from data
        """

        self.Qtable = np.zeros([7999,8])
        self.gamma = 0.7
        self.alpha = 0.2
        self.state = None
        self.action = np.array([-.1,-.1,-.1])

        # seed a random value for now
        self.seed = 80 # 1: 10, 2: 40. 3: 80
        np.random.seed(self.seed)
        self.rand_possible_env = [np.random.randint(low=0,high=2) for _ in range(3)]
        self.rand_possible_goal = [np.random.randint(low=0, high=1) for _ in range(3)]
        self.rand_possible_traj = [np.random.randint(low=1, high=3) for _ in range(3)]

    def get_state(self, env, goal, traj):
        
        tau = open("learning_implement-main/planning/joint_space_full_traj_eef/shortest_path_env_"+str(env)+"_goal_"+str(goal)+"_traj_"+str(traj)+".txt", "r")
        
        return tau
    
    def get_feedback(self, env, goal, traj):
        
        tamer_feedback = open("learning_implement-main/planning/joint_space_full_traj_scalar/shortest_path_env_"+str(env)+"_goal_"+str(goal)+"_traj_"+str(traj)+".txt", "r")

        return tamer_feedback
    
    def get_index(self, state, action, next_state):
        
        """
        action
        """
        action_idx = 0
        if action[0] == .1:
            if action[1] == .1:
                if action[2] == .1:
                    action_idx = 7
                elif action[2] == -.1:
                    action_idx = 6
            elif action[1] == -.1:
                if action[2] == .1:
                    action_idx = 5
                elif action[2] == -.1:
                    action_idx = 4
        elif action[0] == -.1:
            if action[1] == .1:
                if action[2] == .1:
                    action_idx = 3
                elif action[2] == -.1:
                    action_idx = 2
            elif action[1] == -.1:
                if action[2] == .1:
                    action_idx = 1
                elif action[2] == -.1:
                    action_idx = 0
        else:
            print(action) 
            raise Exception('invalid action. Could not convert to index.')
        """
        state index using 10-base binaries, floor estimated
        """
        state = np.asarray(state) + 1.
        new_state = np.array([state[0]/2., state[1]/2., state[2]/2.])
        state_idx = int(10*new_state[0]//0.1/10 + 100*new_state[1]//0.1/10 + 1000*new_state[2]//0.1/10)
        
        """
        next_state using 10-base binaries, floor estimated
        """
        next_state = np.asarray(next_state) + 1.
        new_next_state = np.array([next_state[0]/2., next_state[1]/2., next_state[2]/2.])
        next_state_idx = int(10*new_next_state[0]//0.1/10 + 100*new_next_state[1]//0.1/10 + 1000*new_next_state[2]//0.1/10)
        
        return (state_idx, action_idx, next_state_idx)
    
    def get_next_action(self, current_action, delTraj):

        """
        returns next action based on trajectory residuals
        """
        next_action = current_action
        if delTraj[0] >= 0: next_action[0] = .1
        if delTraj[1] >= 0: next_action[1] = .1
        if delTraj[2] >= 0: next_action[2] = .1

        return next_action
    
    def get_trajectory_residual(self, T, current_iter):
        
        curr_traj = T[current_iter][1:-2].strip()
        curr_traj_list = [float(t) for t in curr_traj.split()]
        next_traj = T[current_iter + 1][1:-2].strip()
        next_traj_list = [float(t) for t in next_traj.split()]

        return np.asarray(next_traj_list) - np.asarray(curr_traj_list)

    
    def get_tamer_binary(self, tamer_binary, current_iter):

        return tamer_binary[current_iter]

    def save_Qtable(self, Qtable):

        with open('Qtable.npy', 'wb') as f:
            np.save(f, Qtable)
    
    def Qtraining(self, envs, goals, trajs):

        for e in envs:
            for g in goals:
                for t in trajs:
                    
                    Trajectory = self.get_state(e, g, t)
                    Tamer_Binaries = self.get_feedback(e, g, t)
                    self.action = np.array([-.1,-.1,-.1])
                    T = Trajectory.readlines()
                    tamer_binary = [float(d.strip()) for d in Tamer_Binaries.readlines()]

                    for i in range(len(T)-1):

                        traj_res = self.get_trajectory_residual(T, i)

                        curr_state = [float(_) for _ in T[i][1:-2].split()]
                        next_state = np.asarray(curr_state) + self.action
                        index_tuple = self.get_index(curr_state, self.action, next_state)
                        state_i = index_tuple[0]
                        action_i = index_tuple[1]
                        state_ip1 = index_tuple[2]
                        h = self.get_tamer_binary(tamer_binary, i)

                        if h != 0:
                            Qvalue = (1 - self.alpha)*self.Qtable[state_i][action_i] + self.alpha*((h - self.Qtable[state_i][action_i]) + self.gamma*self.Qtable[state_ip1][action_i])
                            self.Qtable[state_i][action_i] = Qvalue
                        
                        # update actions
                        self.action = self.get_next_action(self.action, traj_res)
        return 
    
    def exec(self):

        def get_action_from_idx(action_idx):
            if action_idx == 0:
                act = np.array([-.1, -.1, -.1])
            elif action_idx == 1:
                act = np.array([-.1, -.1, .1])
            elif action_idx == 2:
                act = np.array([-.1, .1, -.1])
            elif action_idx == 3:
                act = np.array([-.1, .1, .1])
            elif action_idx == 4:
                act = np.array([.1, -.1, -.1])
            elif action_idx == 5:
                act = np.array([.1, -.1, .1])
            elif action_idx == 6:
                act = np.array([.1, .1, -.1])
            elif action_idx == 7:
                act = np.array([.1, .1, .1])
            
            return act

        prev_state = np.array([0., 0., 0.])
        state = np.array([0.3, 0.2, 0.2])
        goal = np.array([0.642, -0.183, 0.245])
        action = np.array([.1, -.1, .1])
        index_tuple = self.get_index(state, action, state + action)
        state_i = index_tuple[0]
        action_i = index_tuple[1]
        states = []
        actions = []
        print(goal)

        loss = []

        for _ in range(10):
            states.append([x for x in state].copy())
            actions.append(action)

            h = int(input("Update given dist: %s : " % np.linalg.norm(goal - state)))
            if h != 0:
                print("state_i", state_i)
                print("action_i", action_i)
                err = h - self.Qtable[state_i][action_i]
                self.Qtable[state_i][action_i] = (1 - self.alpha)*self.Qtable[state_i][action_i] + self.alpha*(err + self.gamma*self.Qtable[state_i+1][action_i])

            action_i = np.argmax(self.Qtable[state_i])
            action = get_action_from_idx(action_i)

            print("action: ", action)
            loss.append(np.linalg.norm(goal - state))
            state += action
            state_i = self.get_index(state, action, state+action)[0]
        
        states2 = []
        for ele in states:
            ele.append(0.)
            ele.append(0.)
            ele.append(0.)
            ele.append(1.)
            states2.append(ele)

        print(states2)
        return loss

tamer = TAMER()
tamer.Qtraining(tamer.rand_possible_env, 
                tamer.rand_possible_goal, 
                tamer.rand_possible_traj)
tamer.save_Qtable(tamer.Qtable)
print(tamer.Qtable)
loss = tamer.exec()

np.savetxt("loss_tamer.txt", loss, delimiter=',')