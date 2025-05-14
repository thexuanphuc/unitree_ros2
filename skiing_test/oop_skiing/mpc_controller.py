import casadi as ca
import numpy as np
from kinematics import forward_kinematics
from config_skate_board import config_skate_board   

# -------------------------------
# MPC controller for FR leg
# -------------------------------
class MPCController:
    def __init__(self, config):
        self.N_pushing = config["N"]
        self.N_floor = int(0.4 * self.N_pushing)  # number of steps that the foot lie on the floor
        self.N_lift = config["N_lift"]
        self.dt = config["dt"]
        self.fix_hip = config["fix_hip"]
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
        self.dq_vars_pushing = [self.opti_pushing.variable(3) for _ in range(self.N_pushing + 1)]

        # for lifting MPC
        self.opti_lifting = ca.Opti()
        self.q_vars_lifting = [self.opti_lifting.variable(3) for _ in range(self.N_lift + 1)]
        self.dq_vars_lifting = [self.opti_lifting.variable(3) for _ in range(self.N_lift + 1)]

    def setup_problem_pushing(self, cur_robot, side: str = "FR", path_distance: float = 0.0, q0_FR_on_floor: np.ndarray = None):
        """
        this mpc is for pushing the foot on the floor
        path_distance is the coordinate of the path on y axis   

        self.q_vars_pushing is the joint angle
        self.dq_vars_pushing is the joint velocity
        """
        global_hip_offset = getattr(cur_robot, f"global_hip_offset_{side}")

        # Initial position constraint
        self.opti_pushing.subject_to(self.q_vars_pushing[0] == q0_FR_on_floor)

        # Final position constraint to make it move in loop, and zero velocity
        self.opti_pushing.subject_to(self.q_vars_pushing[self.N_pushing] == q0_FR_on_floor)
        self.opti_pushing.subject_to(self.dq_vars_pushing[self.N_pushing] == np.zeros(3))
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
            p_foot = forward_kinematics(self.q_vars_pushing[k], cur_robot, side=side, fix_hip=self.fix_hip).flatten()
            # constraint the length of the foot trajectory
            if k == 0:
                self.x_foot_start = p_foot[0]
            if k == self.N_floor:
                min_push_length = 0.15
                self.opti_pushing.subject_to(self.x_foot_start - p_foot[0] >= min_push_length)

            min_lift_height = 0.04  # 2.5 cm above the ground
        

            if k <= self.N_floor:
                # constraint to keep the path on floor straight
                cost += self.w_y * (p_foot[1] - path_distance) ** 2
                # Ground contact constraint
                self.opti_pushing.subject_to(p_foot[2] == 0)
            else:
                # if k <= int(1.15* self.N_floor):
                    # constraint to keep the path out side the skateboard
                    # cost += self.w_y * (p_foot[1] - path_distance) ** 2
                    # self.opti_pushing.subject_to(p_foot[1] <= path_distance)

                if k > self.N_floor + 45 and k < self.N_pushing - 45:
                    # the part of trajecotry that is not on the floor
                    self.opti_pushing.subject_to(p_foot[2] >= min_lift_height)

        for k in range(self.N_pushing+1):
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

        ipopt_opts = {"print_level": 0, "max_iter": 500, "tol": 1e-4}
        self.opti_pushing.solver('ipopt', {"expand": True}, ipopt_opts)
        for k in range(self.N_pushing):
            self.opti_pushing.set_initial(self.q_vars_pushing[k], q0_FR_on_floor)
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
        dq_sol = np.array([sol.value(self.dq_vars_pushing[k]) for k in range(self.N_pushing + 1)])
        return q_sol, dq_sol

    def setup_problem_lifting(self,cur_robot, side:str="FR", q0_FR_on_board:np.ndarray=None, q0_FR_on_floor:np.ndarray=None):
        """
        this mpc is for lifting the foot from skateboard to the floor
        """
        global_hip_offset = getattr(cur_robot, f"global_hip_offset_{side}")

        # Initial position constraint
        # TODO what is the position of the foot on the skateboard
        self.opti_lifting.subject_to(self.q_vars_lifting[0] == q0_FR_on_board)

        # Final position of lifting phase should the same as the initial position of pushing phase, and zero velocity
        self.opti_lifting.subject_to(self.q_vars_lifting[self.N_lift] == q0_FR_on_floor)
        self.opti_lifting.subject_to(self.dq_vars_lifting[self.N_lift] == np.zeros(3))

        # Dynamics and bounds for each step
        for k in range(self.N_lift):
            # kinematics constraints
            self.opti_lifting.subject_to(self.q_vars_lifting[k + 1] == self.q_vars_lifting[k] + self.dq_vars_lifting[k] * self.dt)

            # limits for joint angles and velocities
            self.opti_lifting.subject_to(self.opti_lifting.bounded(self.q_min, self.q_vars_lifting[k], self.q_max))
            self.opti_lifting.subject_to(self.opti_lifting.bounded(-self.dq_max, self.dq_vars_lifting[k], self.dq_max))

        cost = 0
        # constraint to ensure the foot does not go through the skateboard
        for k in range(int(0.1*self.N_lift), int(0.9*self.N_lift)):
            p_foot = forward_kinematics(self.q_vars_lifting[k], cur_robot, side=side, fix_hip=self.fix_hip).flatten()

            # if the foot[1] < 0.24 (skate board size + 0.04), then foot[2] should be > 0.058 (skateboard height)
            alpha = -1000  # steepness of the sigmoid
            threshold = -config_skate_board["width"] / 2 - 0.04
            min_height = config_skate_board["high"] + 0.02

            # Smooth condition: 0 when p_foot[1] < -0.21, 1 when p_foot[1] > -0.21
            condition = 1 / (1 + ca.exp(alpha * (p_foot[1] - threshold)))  # sigmoid

            # Enforce that if condition is "on", p_foot[2] >= min_height
            penalty = self.w_board_penalty * (ca.fmin(0, p_foot[2] - min_height * condition)) ** 2
            cost += penalty

        for k in range(self.N_lift + 1):
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
    
        ipopt_opts = {"print_level": 4, "max_iter": 500, "tol": 1e-4}
        self.opti_lifting.solver('ipopt', {"expand": True}, ipopt_opts)
        # self.opti_lifting.solver('sqpmethod', {'print_header': True, 'print_time': False, 'print_iteration': True})
        # not to the end, because end position of foot should be on the floor -> constraint violations for the last step
        for k in range(self.N_lift - 5):
            self.opti_lifting.set_initial(self.dq_vars_lifting[k], 0.0)
            self.opti_lifting.set_initial(self.q_vars_lifting[k], q0_FR_on_board)
            # Check if initial guess is feasible
            # print("Constraint violations: ---------------", self.opti_lifting.debug.constraint_violations())

        self.opti_lifting.set_initial(self.q_vars_lifting[self.N_lift], q0_FR_on_board)
        try:
            sol = self.opti_lifting.solve()
            # get the solution for joint angles
            q_sol = np.array([sol.value(self.q_vars_lifting[k]) for k in range(self.N_lift + 1)])
            # get the solution for joint velocities
            dq_sol = np.array([sol.value(self.dq_vars_lifting[k]) for k in range(self.N_lift + 1)])
            return q_sol, dq_sol
        except Exception as e:
            print("Debugging values:")
            print("Parameters:", self.opti_lifting.debug.value(self.parameters))
            print("Objective:", self.opti_lifting.debug.value(self.objective))
            raise e


    def setup_problem_simple_moving_leg(self,cur_robot, side:str="FL", initial_pose:np.ndarray=None, final_pose:np.ndarray=None, N_steps:int=100, min_height:float=0.02):
        """
        this mpc is for lifting the foot from skateboard to the floor
        """
        opti_moving_leg = ca.Opti()
        q_move_legs = [opti_moving_leg.variable(3) for _ in range(N_steps + 1)]
        dq_move_legs = [opti_moving_leg.variable(3) for _ in range(N_steps + 1)]

        # Initial position constraint
        opti_moving_leg.subject_to(q_move_legs[0] == initial_pose)

        # Final position of lifting phase should the same as the initial position of pushing phase, and zero velocity
        opti_moving_leg.subject_to(q_move_legs[N_steps] == final_pose)
        opti_moving_leg.subject_to(dq_move_legs[N_steps] == np.zeros(3))

        # Dynamics and bounds for each step
        for k in range(N_steps):
            # kinematics constraints
            opti_moving_leg.subject_to(q_move_legs[k + 1] == q_move_legs[k] + dq_move_legs[k] * self.dt)

            # limits for joint angles and velocities
            opti_moving_leg.subject_to(opti_moving_leg.bounded(self.q_min, q_move_legs[k], self.q_max))
            opti_moving_leg.subject_to(opti_moving_leg.bounded(-self.dq_max, dq_move_legs[k], self.dq_max))

        cost = 0
        # constraint to ensure the foot does not go through the skateboard
        for k in range(int(0.15 * N_steps), int(0.85 * N_steps)):
            p_foot = forward_kinematics(q_move_legs[k], cur_robot, side=side, fix_hip=self.fix_hip).flatten()

            # just try to lift the foot            
            penalty = self.w_board_penalty * (ca.fmin(0, p_foot[2] - min_height)) ** 2
            cost += penalty

        for k in range(N_steps + 1):
            # this cost to minimize the velocity
            cost += self.w_dq * ca.sumsqr(dq_move_legs[k] - 0)
            if k > 0:
                # this cost to smooth the velocity
                # TODO put into threshold function for velocity, not just zeros, that velocity should be smaller than some value
                penalty = 10 * self.w_smooth * (dq_move_legs[k][0] - dq_move_legs[k - 1][0]) ** 2 \
                        + 2 * self.w_smooth * (dq_move_legs[k][1] - dq_move_legs[k - 1][1]) ** 2 \
                        + self.w_smooth * (dq_move_legs[k][2] - dq_move_legs[k - 1][2]) ** 2  # different weights for hip, knee, ankle
        
                cost += penalty
        opti_moving_leg.minimize(cost)
    
        ipopt_opts = {"print_level": 4, "max_iter": 500, "tol": 1e-4}
        opti_moving_leg.solver('ipopt', {"expand": True}, ipopt_opts)
        # not to the end, because end position of foot should be on the floor -> constraint violations for the last step
        for k in range(N_steps - 5):
            opti_moving_leg.set_initial(dq_move_legs[k], 0.0)
            opti_moving_leg.set_initial(q_move_legs[k], initial_pose)

        opti_moving_leg.set_initial(q_move_legs[N_steps], final_pose)
        try:
            sol = opti_moving_leg.solve()
            # get the solution for joint angles
            q_sol = np.array([sol.value(q_move_legs[k]) for k in range(N_steps + 1)])
            # get the solution for joint velocities
            dq_sol = np.array([sol.value(dq_move_legs[k]) for k in range(N_steps + 1)])
            return q_sol, dq_sol
        except Exception as e:
            print("Debugging values:")
            print("Parameters:", opti_moving_leg.debug.value(self.parameters))
            print("Objective:", opti_moving_leg.debug.value(self.objective))
            raise e


