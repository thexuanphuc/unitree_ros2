import casadi as ca
import numpy as np

# -------------------------------
# MPC controller for FR leg
# -------------------------------
class MPCController:
    def __init__(self, robot, config, q0_FR_on_board, q0_FR_on_floor):
        self.robot = robot
        self.N_pushing = config["N"]
        self.N_floor = int(0.4 * self.N_pushing)  # number of steps that the foot lie on the floor
        self.N_lift = config["N_lift"]
        self.dt = config["dt"]
        self.fix_hip = config["fix_hip"]
        self.q0_FR_on_board = q0_FR_on_board  # initial position for lifting part (from the skateboard)
        self.q0_FR_on_floor = q0_FR_on_floor  # initial position for pushing part (on the floor)
        self.dq_max = config["dq_max"]
        self.w_pos = config["w_pos"]
        self.w_dq = config["w_dq"]
        self.w_smooth = config["w_smooth"]
        self.w_y = config["w_y"]
        # TODO move to config
        self.w_board_penalty = 100000.0
        self.q_min = np.array(config["joint_limits"]["q_min"])
        self.q_max = np.array(config["joint_limits"]["q_max"])

        # for pushing MPC
        self.opti_pushing = ca.Opti()
        self.q_vars_pushing = [self.opti_pushing.variable(3) for _ in range(self.N_pushing + 1)]
        self.dq_vars_pushing = [self.opti_pushing.variable(3) for _ in range(self.N_pushing)]

        # for lifting MPC
        self.opti_lifting = ca.Opti()
        self.q_vars_lifting = [self.opti_lifting.variable(3) for _ in range(self.N_lift + 1)]
        self.dq_vars_lifting = [self.opti_lifting.variable(3) for _ in range(self.N_lift)]

    def setup_problem_pushing(self, effective_hip_offset, distance_from_path_to_center):
        """
        this mpc is for pushing the foot on the floor
        self.q_vars_pushing is the joint angle

        TODO: 
            + add constraints for end position
        """
        print("the intial position of the foot on the skateboard:", self.q0_FR_on_floor)
        print("the angle limits for the joints:", self.q_min, self.q_max)

        # Initial position constraint
        self.opti_pushing.subject_to(self.q_vars_pushing[0] == self.q0_FR_on_floor)

        # Final position constraint to make it move in loop
        self.opti_pushing.subject_to(self.q_vars_pushing[self.N_pushing] == self.q0_FR_on_floor)

        # Dynamics and bounds for each step
        for k in range(self.N_pushing):
            # kinematics constraints
            self.opti_pushing.subject_to(self.q_vars_pushing[k + 1] == self.q_vars_pushing[k] + self.dq_vars_pushing[k] * self.dt)

            # limits for joint angles and velocities
            self.opti_pushing.subject_to(self.opti_pushing.bounded(self.q_min, self.q_vars_pushing[k], self.q_max))
            self.opti_pushing.subject_to(self.opti_pushing.bounded(-self.dq_max, self.dq_vars_pushing[k], self.dq_max))

        # Final position bounds
        self.opti_pushing.subject_to(self.opti_pushing.bounded(self.q_min, self.q_vars_pushing[self.N_pushing], self.q_max))

        cost = 0

        # constraint for trajectory of the foot on the floor
        for k in range(self.N_pushing + 1):
            p_foot = forward_kinematics(self.q_vars_pushing[k], self.robot, effective_hip_offset, self.fix_hip)
            
            # constraint the length of the foot trajectory
            if k == 0:
                self.x_foot_start = p_foot[0]
            if k == self.N_floor:
                min_push_length = 0.15
                self.opti_pushing.subject_to(self.x_foot_start - p_foot[0] >= min_push_length)

            min_lift_height = 0.025  # 5 cm above the ground
        
            if k <= self.N_floor:
                # constraint to keep the path on floor straight
                cost += self.w_y * (p_foot[1] - distance_from_path_to_center) ** 2
                # Ground contact constraint
                self.opti_pushing.subject_to(p_foot[2] == 0)

            elif k > self.N_floor + 20 and k < self.N_pushing - 20:
                # the part of trajecotry that is not on the floor
                self.opti_pushing.subject_to(p_foot[2] >= min_lift_height)

        for k in range(self.N_pushing):
            # this cost to minimize the velocity
            # TODO put into threshold function for velocity, not just zeros, that velocity should be smaller than some value
            cost += self.w_dq * ca.sumsqr(self.dq_vars_pushing[k] - 0)
            
            if k > 0:
                # this cost to smooth the velocity
                # TODO put into threshold function for velocity, not just zeros, that velocity should be smaller than some value
                penalty = 1 * self.w_smooth * (self.dq_vars_pushing[k][0] - self.dq_vars_pushing[k - 1][0]) ** 2 \
                        + 0.5 * self.w_smooth * (self.dq_vars_pushing[k][1] - self.dq_vars_pushing[k - 1][1]) ** 2 \
                        + self.w_smooth * (self.dq_vars_pushing[k][2] - self.dq_vars_pushing[k - 1][2]) ** 2  # different weights for hip, knee, ankle
                cost += penalty

        self.opti_pushing.minimize(cost)

    def solve_pushing(self):
        ipopt_opts = {"print_level": 0, "max_iter": 500, "tol": 1e-4}
        self.opti_pushing.solver('ipopt', {"expand": True}, ipopt_opts)
        for k in range(self.N_pushing):
            self.opti_pushing.set_initial(self.q_vars_pushing[k], self.q0_FR_on_floor)
            self.opti_pushing.set_initial(self.dq_vars_pushing[k], 0.0)
        try:
            sol = self.opti_pushing.solve()
        except RuntimeError as e:
            print("Solver failed!")
            print("q0 initial:", self.opti_pushing.debug.value(self.q_vars_pushing[0]))
            print("qN guess:", self.opti_pushing.debug.value(self.q_vars_pushing[-1]))
            raise e
        
        # get the solution for joint angles
        q_sol = np.array([sol.value(self.q_vars_pushing[k]) for k in range(self.N_pushing + 1)])

        # get the solution for joint velocities
        dq_sol = np.array([sol.value(self.dq_vars_pushing[k]) for k in range(self.N_pushing)])
        return q_sol, dq_sol

    def setup_problem_lifting(self, effective_hip_offset):
        """
        this mpc is for lifting the foot from skateboard to the floor
        """
        # Initial position constraint
        # TODO what is the position of the foot on the skateboard
        self.opti_lifting.subject_to(self.q_vars_lifting[0] == self.q0_FR_on_board)

        # Final position of lifting phase should the same as the initial position of pushing phase
        self.opti_lifting.subject_to(self.q_vars_lifting[self.N_lift] == self.q0_FR_on_floor)

        # Dynamics and bounds for each step
        for k in range(self.N_lift):
            # kinematics constraints
            self.opti_lifting.subject_to(self.q_vars_lifting[k + 1] == self.q_vars_lifting[k] + self.dq_vars_lifting[k] * self.dt)

            # limits for joint angles and velocities
            self.opti_lifting.subject_to(self.opti_lifting.bounded(self.q_min, self.q_vars_lifting[k], self.q_max))
            self.opti_lifting.subject_to(self.opti_lifting.bounded(-self.dq_max, self.dq_vars_lifting[k], self.dq_max))

        cost = 0
        # constraint to ensure the foot does not go through the skateboard
        for k in range(self.N_lift + 1):
            p_foot = forward_kinematics(self.q_vars_lifting[k], self.robot, effective_hip_offset, self.fix_hip)
            
            # if the foot[1] < 0.24 (skate board size + 0.04), then foot[2] should be > 0.058 (skateboard height)
            alpha = -1000  # steepness of the sigmoid
            threshold = -0.21
            min_height = 0.06

            # Smooth condition: 0 when p_foot[1] < -0.21, 1 when p_foot[1] > -0.21
            condition = 1 / (1 + ca.exp(alpha * (p_foot[1] - threshold)))  # sigmoid

            # Enforce that if condition is "on", p_foot[2] >= min_height
            penalty = self.w_board_penalty * (ca.fmin(0, p_foot[2] - min_height * condition)) ** 2
            cost += penalty

        for k in range(self.N_lift):
            # this cost to minimize the velocity
            # TODO put into threshold function, that velocity should be smaller than some value
            # but do we really need this one???
            cost += self.w_dq * ca.sumsqr(self.dq_vars_lifting[k] - 0)
            if k > 0:
                # this cost to smooth the velocity
                # TODO put into threshold function for velocity, not just zeros, that velocity should be smaller than some value
                penalty = 10 * self.w_smooth * (self.dq_vars_lifting[k][0] - self.dq_vars_lifting[k - 1][0]) ** 2 \
                        + 2 * self.w_smooth * (self.dq_vars_lifting[k][1] - self.dq_vars_lifting[k - 1][1]) ** 2 \
                        + self.w_smooth * (self.dq_vars_lifting[k][2] - self.dq_vars_lifting[k - 1][2]) ** 2  # different weights for hip, knee, ankle
        
                cost += penalty
        self.opti_lifting.minimize(cost)
    
    def solve_lifting(self):
        ipopt_opts = {"print_level": 0, "max_iter": 500, "tol": 1e-4}
        self.opti_lifting.solver('ipopt', {"expand": True}, ipopt_opts)
        for k in range(self.N_lift):
            self.opti_lifting.set_initial(self.dq_vars_lifting[k], 0.0)
            self.opti_lifting.set_initial(self.q_vars_lifting[k], self.q0_FR_on_board)
        self.opti_lifting.set_initial(self.q_vars_lifting[self.N_lift], self.q0_FR_on_board)
        
        sol = self.opti_lifting.solve()
        # get the solution for joint angles
        q_sol = np.array([sol.value(self.q_vars_lifting[k]) for k in range(self.N_lift + 1)])
        # get the solution for joint velocities
        dq_sol = np.array([sol.value(self.dq_vars_lifting[k]) for k in range(self.N_lift)])
        return q_sol, dq_sol