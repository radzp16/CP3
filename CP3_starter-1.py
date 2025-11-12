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

        orn = p.getQuaternionFromEuler([0, -3.141*3/2, 0])
        # move to a point given 

        poses = p.calculateInverseKinematics(self.pandaId, self.end_effector_index, point, orn)

        for i in range(9):
            p.resetJointState(self.pandaId, i, poses[i])

        return
    
    def find_loc_difference(self, point, blackboard=True):

        if blackboard:
            point = [0.5] + list(point)

        link_state = p.getLinkState(self.pandaId, self.end_effector_index, computeForwardKinematics=True)
        link_position = link_state[0]
        tip = list(link_position)
        
        difference = ((tip[0] - point[0])**2 + 
                      (tip[1] - point[1])**2 + (tip[2] - point[2])**2)**0.5
        
        return difference







def make_c_points(base_pt):
    y = np.linspace(base_pt, base_pt+0.38, 15)
    eq = lambda y: (0.25**2 - (y - base_pt)**2)**0.5
    z_plus = .5 + eq(y)
    z_minus = .5 - eq(y)

    y_z_plus = np.column_stack((y, z_plus))
    y_z_minus = np.column_stack((y, z_minus))

    points = np.concatenate((y_z_plus, y_z_minus))

    print(points)

    return points

def make_line(base_pt):
    z = np.linspace(base_pt, base_pt+0.2, 10)


    


if __name__ == "__main__":
    pandaId = setup_pybullet()
    panda = Panda(pandaId)
    print("Done setting up pybullet")

    p.setRealTimeSimulation(0)
    # Start recording video 
    # log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./video.mp4")

    c_points = make_c_points(0)

    while p.isConnected():
        
        # IMPORTANT - You need to run this command for every step in simulation
        for point in c_points:
            panda.move_to_point(point)
            
            distance = panda.find_loc_difference(point)

            while distance > 0.01:
                panda.move_to_point(point)
                distance = panda.find_loc_difference(point)
                p.stepSimulation()
            panda.draw_on_blackboard() 
        # Command to stop recording when done
        # p.stopStateLogging(log_id)

    p.disconnect()