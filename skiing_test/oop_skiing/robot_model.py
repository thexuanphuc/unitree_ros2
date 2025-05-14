import xml.etree.ElementTree as ET
import numpy as np

# -------------------------------
# Configuration
# -------------------------------
default_config = {
    "urdf_file": "a1.urdf",
    "standing_height": 0.35,      # approximate foot height
    "step_length": 0.2,
    "swing_height": 0.07,
    "trajectory_type": "two_phase",
    "N": 500,
    "N_lift": 200,  # number of steps for lifting one foot from skateboard to the floor
    "dt": 0.01,
    "dq_max": 28.6,
    "fix_hip": False,             # in MPC mode fix hip; for IK - don't fix q[0]
    "q0": [0.0, 0.0, -0.9],
    "w_pos": 10000.0,
    "w_dq": 0.001,
    "w_smooth": 0.1,
    "w_y": 1000.0,
    "joint_limits": {
         "q_min": [-0.80, -1.05, -2.70],
         "q_max": [ 0.80,  4.19, -0.92]
    },
    # Parameter for shifting the center of mass (trunk) in com_transfering mode
    "com_shifting": 0.055,
    # number of steps for shifting the center of mass
    "N_lean": 100, 
}

hip_target = np.deg2rad(0)  # initial hip-roll value (in radians)

# -------------------------------
# Function to read offset from URDF by joint name
# -------------------------------
def read_offset(urdf_file, joint_name):
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    for joint in root.findall("joint"):
        if joint.get("name") == joint_name:
            origin = joint.find("origin")
            if origin is not None and "xyz" in origin.attrib:
                return np.array([float(v) for v in origin.attrib["xyz"].split()])
    return None

# -------------------------------
# Robot model class (reading URDF and kinematics)
# -------------------------------
class RobotModel:
    def __init__(self, config):
        self.urdf_file = config["urdf_file"]
        self.L1, self.L2, self.L3 = self.read_link_lengths_from_urdf()
        self.trunk_dims = self.read_trunk_dimensions_from_urdf()
        self.hip_offset_FR = read_offset(self.urdf_file, "FR_hip_joint")
        self.hip_fixed_offset_FR = read_offset(self.urdf_file, "FR_hip_fixed")
        self.hip_offset_FL = read_offset(self.urdf_file, "FL_hip_joint")
        self.hip_fixed_offset_FL = read_offset(self.urdf_file, "FL_hip_fixed")
        self.hip_offset_RR = read_offset(self.urdf_file, "RR_hip_joint")
        self.hip_fixed_offset_RR = read_offset(self.urdf_file, "RR_hip_fixed")
        self.hip_offset_RL = read_offset(self.urdf_file, "RL_hip_joint")
        self.hip_fixed_offset_RL = read_offset(self.urdf_file, "RL_hip_fixed")
        self.link_masses = self.read_link_masses_from_urdf()


        # the hip_offset_FL is offset from global(center now) to the hip joint
        # the hip_fixed_offset_FL is offset from the hip joint to the thigh joint (on y axis)
        # the robot.hip_offset_FL is [0.183 0.047 0.   ]
        # the robot.hip_fixed_offset_FL is [0.    0.081 0.   ]
                
    def read_link_lengths_from_urdf(self):
        tree = ET.parse(self.urdf_file)
        root = tree.getroot()
        L1, L2, L3 = None, None, None
        for joint in root.findall("joint"):
            if joint.get("name") == "FR_hip_fixed":
                origin = joint.find("origin")
                if origin is not None and "xyz" in origin.attrib:
                    values = [float(v) for v in origin.attrib["xyz"].split()]
                    L1 = np.linalg.norm(values)
                break
        for link in root.findall("link"):
            if link.get("name") == "FR_thigh":
                collision = link.find("collision")
                if collision is not None:
                    geometry = collision.find("geometry")
                    if geometry is not None:
                        box = geometry.find("box")
                        if box is not None and "size" in box.attrib:
                            sizes = [float(v) for v in box.attrib["size"].split()]
                            L2 = max(sizes)
                break
        for link in root.findall("link"):
            if link.get("name") == "FR_calf":
                collision = link.find("collision")
                if collision is not None:
                    geometry = collision.find("geometry")
                    if geometry is not None:
                        box = geometry.find("box")
                        if box is not None and "size" in box.attrib:
                            sizes = [float(v) for v in box.attrib["size"].split()]
                            L3 = max(sizes)
                break
        if L1 is None or L2 is None or L3 is None:
            print("Error reading link lengths from URDF - using default values.")
            L1, L2, L3 = 0.085, 0.20, 0.20
        print("Link lengths: L1 = {:.3f}, L2 = {:.3f}, L3 = {:.3f}".format(L1, L2, L3))
        return L1, L2, L3

    def read_trunk_dimensions_from_urdf(self):
        tree = ET.parse(self.urdf_file)
        root = tree.getroot()
        trunk_dims = None
        for link in root.findall("link"):
            if link.get("name") == "trunk":
                collision = link.find("collision")
                if collision is not None:
                    geometry = collision.find("geometry")
                    if geometry is not None:
                        box = geometry.find("box")
                        if box is not None and "size" in box.attrib:
                            trunk_dims = [float(v) for v in box.attrib["size"].split()]
                break
        if trunk_dims is None:
            trunk_dims = [0.267, 0.194, 0.114]
        print("Trunk dimensions (x,y,z):", trunk_dims)
        return trunk_dims

    def read_link_masses_from_urdf(self):
        tree = ET.parse(self.urdf_file)
        root = tree.getroot()
        masses = {}
        for link in root.findall("link"):
            mass = None
            inertial = link.find("inertial")
            if inertial is not None:
                mass_elem = inertial.find("mass")
                if mass_elem is not None and "value" in mass_elem.attrib:
                    mass = float(mass_elem.attrib["value"])
            if mass is not None:
                masses[link.get("name")] = mass
        print("Link masses from URDF:", masses)
        return masses
    
    def set_trunk_center(self, trunk_center:np.ndarray):
        # trunk_center: [x, y, z]

        self.trunk_center = trunk_center
        self.global_hip_offset_FR = self.trunk_center + (self.hip_offset_FR)
        self.global_hip_offset_FL = self.trunk_center + (self.hip_offset_FL)
        self.global_hip_offset_RR = self.trunk_center + (self.hip_offset_RR)
        self.global_hip_offset_RL = self.trunk_center + (self.hip_offset_RL)
        print("the center of trunk is ------------------->", self.trunk_center)
        print("global_hip_offset_FL was recalculated ------------------->", self.global_hip_offset_FL)
