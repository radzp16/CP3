import pybullet as p
import pybullet_data
import numpy as np
import time


### Setup Pybullet ###

def setup_pybullet():
    # We don't use this but we need it.
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=-90, cameraPitch=-20, cameraTargetPosition=[0.5, 0.2, 1.0])
    planeId = p.loadURDF("plane.urdf")
    startPos = [0, 0, 0]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    pandaId = p.loadURDF("franka_panda/panda.urdf", startPos, startOrientation, useFixedBase=1)
    blackboard_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 1, 0.5]),
        baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 1, 0.5], rgbaColor=[0, 0, 0, 1]),
        basePosition=[0.5, 0.5, 0.5]
    )
    # Note: the blackboard is located at x=0.5, y=0.5

    return pandaId


### Panda ###

class Panda:
    """
    This class sets up your manipulator. It includes one built in function draw_on_blackboard.
    You are free to add any additional functions if you'd like.
    """
    def __init__(self, pandaId):
        self.pandaId = pandaId
        self.end_effector_index = 11
        self.ll, self.ul, self.jr, self.js = self.get_joint_dynamics()
        self.js = (-0.42345759824061124, 1.6947571434990787, -0.2319733123876026, 1.5292328610779409, 0.39579087373147126, 1.8138928302630748, -2.208256613520994, 0.0, 0.0)



    def get_joint_dynamics(self):

        lower_joint_limits = []
        upper_joint_limits = []
        joint_ranges = []
        pref_state = []
        num_joints = p.getNumJoints(self.pandaId)

        
        for i in range(num_joints):
            info = p.getJointInfo(self.pandaId, i)
            state = p.getJointState(self.pandaId, i)
            if info[2] != 4:
                lower_joint_limits.append(info[8])
                upper_joint_limits.append(info[9])
                joint_ranges.append(info[9] - info[8])
                pref_state.append(state[0])




        return lower_joint_limits, upper_joint_limits, joint_ranges, pref_state


    def draw_on_blackboard(self):
        """
        This function draws a point in 3D space at the current end-effector of the robot
        """
        
        link_state = p.getLinkState(self.pandaId, self.end_effector_index, computeForwardKinematics=True)

        # Position andd orientation of the link in the base frame.
        link_position = link_state[0]
        tip_position = list(link_position)
        tip_position[0] -=0.01 # in case they solve directly on black board, help this display
        vid = p.createVisualShape(p.GEOM_SPHERE, radius=0.01, rgbaColor=[255, 255, 255, 1])
        p.createMultiBody(baseVisualShapeIndex=vid, basePosition=tip_position)

    def move_to_point(self, point, blackboard=True):

        if blackboard:
            point = [0.5] + list(point)
        
        if np.any(np.isnan(point)):
            print(f"Skipped {point}")
            return

        orn = p.getQuaternionFromEuler([0, -3.141*3/2, 0])
        # move to a point given 

        difference = 1
        i = 0
        while difference > 0.01 and i < 5:
            poses = p.calculateInverseKinematics(self.pandaId, self.end_effector_index, point, orn,
                                                 self.ll, self.ul, self.jr, self.js,
                                                 maxNumIterations=100)
            print(self.jr)
            print(self.js)
            print(poses)
            
            
            
            for i in range(9):
                p.resetJointState(self.pandaId, i, poses[i])
            p.stepSimulation()
            difference = self.find_loc_difference(point)
            
            time.sleep(2)

            
            


        p.stopStateLogging(log_id)
        6/0
        return
    
    def find_loc_difference(self, point):


        link_state = p.getLinkState(self.pandaId, self.end_effector_index, computeForwardKinematics=True)
        link_position = link_state[0]
        tip = list(link_position)
        
        difference = ((tip[0] - point[0])**2 + 
                      (tip[1] - point[1])**2 + (tip[2] - point[2])**2)**0.5
        
        return difference







def make_c_points(base_pt):
    r = 0.13
    y = np.linspace(base_pt-r+0.03, base_pt+r, 20, endpoint=False)
    eq = lambda y: (r**2 - (y - base_pt)**2)**0.5
    z_plus = 0.6 + eq(y)
    z_minus = 0.6 - eq(y)

    y_z_plus = np.column_stack((y, z_plus))
    y_z_minus = np.column_stack((y, z_minus))

    points = np.concatenate((y_z_plus, y_z_minus))



    return points

def make_u_points(base_pt_y, base_pt_z):
    r = 0.07
    z = np.linspace(base_pt_z+0.16, base_pt_z-0.005, 10, endpoint=True)
    y = np.array([base_pt_y - r]*len(z))
    y2 = y + r*2

    line1 = np.column_stack((y, z))
    line2 = np.column_stack((y2, z))

    y_circ = np.linspace(base_pt_y-r, base_pt_y+r, 15, endpoint=True)
    eq = lambda y: (r**2 - (y - base_pt_y)**2)**0.5
    z_circ = base_pt_z - eq(y_circ)

    circ = np.column_stack((y_circ, z_circ))

    points = np.concatenate((line1, circ, line2))

    return points

    



    


if __name__ == "__main__":
    pandaId = setup_pybullet()
    panda = Panda(pandaId)
    print("Done setting up pybullet")

    p.setRealTimeSimulation(0)
    # Start recording video 
    log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./video.mp4")

    c_points = make_c_points(0.06)

    u_points = make_u_points(-0.22, 0.57)

    while p.isConnected():

        
        # IMPORTANT - You need to run this command for every step in simulation
        for point in c_points:
            panda.move_to_point(point)        
            panda.draw_on_blackboard()
        for point in u_points:
            panda.move_to_point(point)        
            panda.draw_on_blackboard() 
        # Command to stop recording when done
        # p.stopStateLogging(log_id)

    p.disconnect()