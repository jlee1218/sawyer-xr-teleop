#!/usr/bin/env python

# Small example to move the Sawyer EE position using the keyboard input

import numpy as np

import argparse
import math

import rospy
from std_msgs.msg import String

import intera_interface
import intera_external_devices
# import intera_control

import tf

from intera_interface import CHECK_VERSION

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import EndpointState
from sawyer_xr_teleop.msg import RobotData, ControllerData

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
    SolvePositionFK,
    SolvePositionFKRequest,
)

from copy import deepcopy

#FYDP: Sawyer and Leap limits for transform
X_POS_MAX = 0.84
X_POS_MIN = -0.69
Y_POS_MAX = -0.20
Y_POS_MIN = -1.05
Z_POS_MAX = 0.90
Z_POS_MIN = 0.00

LEAP_X_MIN = -200
LEAP_X_MAX = 200
LEAP_Y_MIN = -200
LEAP_Y_MAX = 200
LEAP_Z_MIN = 50
LEAP_Z_MAX = 500

X_SCALE = (X_POS_MAX - X_POS_MIN) / (LEAP_X_MAX - LEAP_X_MIN)
Y_SCALE = (Y_POS_MAX - Y_POS_MIN) / (LEAP_Y_MAX - LEAP_Y_MIN)
Z_SCALE = (Z_POS_MAX - Z_POS_MIN) / (LEAP_Z_MAX - LEAP_Z_MIN)

MAX_VEL = 0.05
RATE = 10.0

ENDPOINT_EFFORT_HIST_LEN = 25

# FYDP: Sawyer Process Publisher
#global processPub
#processPub = rospy.Publisher('robot_data_message', String, queue_size=10)

# message publisher
pub = rospy.Publisher('robot_motion_sender', String, queue_size=1)

# position_PID_p = [1000.0, 800.0, 500.0, 600.0, 600.0, 200.0, 100.0]
# position_PID_i = [10.0, 10.0, 10.0, 10.0, 10.0, 1.0, 10.0]
# position_PID_d = [25.0, 30.0, 30.0, 50.0, 8.0, 5.0, 4.0]

#FYDP: Map coords
def coord_transform(leap_x, leap_y, leap_z):
    
    leap_x = max(LEAP_X_MIN, min(leap_x, LEAP_X_MAX))
    leap_y = max(LEAP_Y_MIN, min(leap_y, LEAP_Y_MAX))
    leap_z = max(LEAP_Z_MIN, min(leap_z, LEAP_Z_MAX))

    mappedX = X_POS_MAX - (leap_x - LEAP_X_MIN) * X_SCALE 
    mappedY = (leap_y - LEAP_Y_MIN) * Y_SCALE + Y_POS_MIN
    mappedZ = (leap_z - LEAP_Z_MIN) * Z_SCALE + Z_POS_MIN

    return mappedX, mappedY, mappedZ

def calculate_delta(current_ee_pose, transX, transY, transZ):
    return np.linalg.norm([current_ee_pose.position.x - transX, current_ee_pose.position.y - transY, current_ee_pose.position.z - transZ],2)

def check_within_bounds(leap_x, leap_y, leap_z):
    if leap_x > LEAP_X_MAX:
        return 4
    elif leap_x < LEAP_X_MIN:
        return 3
    elif leap_y > LEAP_Y_MAX:
        return 1
    elif leap_y < LEAP_Y_MIN:
        return 2
    elif leap_z > LEAP_Z_MAX:
        return 6
    elif leap_z < LEAP_Z_MIN:
        return 5
    else:
        return 0

# returns the current EE pose peforming FK
def get_current_ee_pose(sawyer_limb, fksvc):

    fkreq = SolvePositionFKRequest()
    joints = JointState()
    joints.name = sawyer_limb.joint_names()
    fkreq.tip_names.append('right_hand')

    # get current joints configuration
    for joint in joints.name:
        joints.position.append(sawyer_limb.joint_angle(joint))

    fkreq.configuration.append(joints)

    # compute FK
    try:
        # rospy.wait_for_service(ns, 5.0)
        resp = fksvc(fkreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False

    # Check if result valid
    if resp.isValid[0]:
        current_pose = resp.pose_stamp[0].pose
    else:
        return False

    return current_pose


def compute_ik(desired_ee_pose, iksvc):

    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    posed = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=desired_ee_pose.position.x,
                    y=desired_ee_pose.position.y,
                    z=desired_ee_pose.position.z,
                ),
                orientation=Quaternion(
                    x=desired_ee_pose.orientation.x,
                    y=desired_ee_pose.orientation.y,
                    z=desired_ee_pose.orientation.z,
                    w=desired_ee_pose.orientation.w,
                ),
            ),
        )

    # Add desired pose for inverse kinematics
    ikreq.pose_stamp.append(posed)

    # Request inverse kinematics from base to "right_hand" link
    ikreq.tip_names.append('right_hand')
    try:
        # rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False
    # Check if result valid, and type of seed ultimately used to get solution
    if resp.result_type[0] > 0:
        # rospy.loginfo("SUCCESS - Valid Joint Solution Found!")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
    else:
        rospy.loginfo("INVALID POSE - No Valid Joint Solution Found.")
        return False

    return limb_joints



def move_ee(side, rate):
    sawyer_arm = intera_interface.Limb(side)
    joint_names = sawyer_arm.joint_names()
    wrist_angle = sawyer_arm.joint_angle(joint_names[6])

    # FYDP Neutral Pose
    sawyer_neutral_pose = [-1.48, -0.97, -0.10, 1.61, -2.98, -0.78, -1.54]

    robot_msg = RobotData()
    robot_msg.movement_obstruction = False


    # reset robot pose to neutral pose
    def reset_pose(limb):
        new_limb_pose = {}
        i = 0
        for joint in joint_names:
            new_limb_pose[joint] = sawyer_neutral_pose[i]
            i += 1
        limb.move_to_joint_positions(new_limb_pose)

    # try:
    #     gripper = intera_interface.Gripper(side + '_gripper')
    # except:
    #     has_gripper = False
    #     rospy.loginfo("The electric gripper is not detected on the robot.")
    # else:
    #     has_gripper = True
    #     rospy.loginfo("The electric gripper is mounted on the robot.")

    # # The gripper has to be calibrated before use
    # if has_gripper:
    #     gripper.calibrate()
    #     if gripper.is_calibrated():
    #         rospy.loginfo("The gripper has been calibrated.")
    #     else:
    #         rospy.loginfo("[WARNING]: the gripper could not be calibrated correctly. Issues may occur.")

    #     # this returns the position of the gripper as how much it is open, in meters
    #     gripper_position = gripper.get_position()
    #     print "Gripper position: " + str(gripper_position)

    # Move to initial position
    reset_pose(sawyer_arm)

    # Start FK service
    nsfk = "ExternalTools/" + side + "/PositionKinematicsNode/FKService"
    fksvc = rospy.ServiceProxy(nsfk, SolvePositionFK)

    # Start IK service
    nsik = "ExternalTools/" + side + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(nsik, SolvePositionIK)

    # Get initial EE pose
    current_ee_pose = get_current_ee_pose(sawyer_arm, fksvc)
    if not current_ee_pose:
        rospy.logerr("Simple FK call FAILED")
    else:
        print "Initial EE pose:\n" + str(current_ee_pose)

    # Initially set the new pose to the current pose
    new_ee_pose = current_ee_pose

    # Move with keyboard
    delta = 0.01 #input("Please choose a delta for the motion: ")
    delta_wrist = 0.01 #input("Please choose a joint delta for the wrist motion: ")
    delta_wrist = math.radians(delta_wrist)
    instructions = """
    Instructions:
     - 1: increase X
     - q: decrease X
     - 2: increase Y
     - w: decrease Y
     - 3: increase Z
     - e: decrease Z
     - 4: increase rot
     - r: decrease rot
    """
    print instructions

    # function to call IK and move the joints to the new obtained pose with raw position control
    def set_ee_pose(limb, ee_pose, iksvc):
        new_limb_pose = compute_ik(ee_pose, iksvc)

        if not new_limb_pose:
            return False
        else:
	    # FYDP line to change
	    # Linear interpolation by default
            limb.set_joint_positions(new_limb_pose)
            
            #FYDP Added: Print Current Pose after each move
            current_ee_pose = get_current_ee_pose(sawyer_arm, fksvc)
            if not current_ee_pose:
                rospy.logerr("Simple FK call FAILED")
            else:
                # print "Current EE pose:\n" + str(current_ee_pose)
                pass

            return True

    # set the joint angle of the wrist joint (last joint)
    def set_wrist_pos(limb, wrist_pos):
        new_limb_pose = {}
        for joint in joint_names:
            new_limb_pose[joint] = limb.joint_angle(joint)
        new_limb_pose[joint_names[6]] = wrist_pos
        limb.set_joint_positions(new_limb_pose)

    bindings = {
        '1': (set_ee_pose, "X increase"),
        'q': (set_ee_pose, "X decrease"),
        '2': (set_ee_pose, "Y increase"),
        'w': (set_ee_pose, "Y decrease"),
        '3': (set_ee_pose, "Z increase"),
        'e': (set_ee_pose, "Z decrease"),
        '4': (set_wrist_pos, "rot increase"),
        'r': (set_wrist_pos, "rot decrease"),
        'x': (reset_pose, "reset robot pose"),
    }

    done = False
    reset_robot_pose = False



    # FYDP: Leap Callback
    def leapCallback(data):
        error = data.error_type


        if error == 0:
            leap_x_coords = data.pos_x
            leap_y_coords = data.pos_y
            leap_z_coords = data.pos_z

            bound_error = check_within_bounds(leap_x_coords, leap_y_coords, leap_z_coords)

            robot_msg.at_edge_limit = bound_error

            transX, transY, transZ = coord_transform(leap_x_coords, leap_y_coords, leap_z_coords)

            current_ee_pose = get_current_ee_pose(sawyer_arm, fksvc)

            delta = calculate_delta(current_ee_pose, transX, transY, transZ)

            if delta > 0.4:
                robot_msg.slow_down = True
            else:
                robot_msg.slow_down = False

            # Only set new pose if ee difference is big enough
            if delta > 0.001 :
                new_ee_pose = current_ee_pose

                new_ee_pose.position.x = transX
                new_ee_pose.position.y = transY
                new_ee_pose.position.z = transZ


                if not set_ee_pose(sawyer_arm, new_ee_pose, iksvc):
                    print("Error")
                    robot_msg.error_type = 1
                else:
                    robot_msg.error_type = 0

            current_ee_pose = get_current_ee_pose(sawyer_arm, fksvc)

            if calculate_delta(current_ee_pose, transX, transY, transZ) < 0.01:
                robot_msg.in_final_position = 1
            else:
                robot_msg.in_final_position = 0

            robot_msg.robot_x_pos = current_ee_pose.position.x
            robot_msg.robot_y_pos = current_ee_pose.position.y
            robot_msg.robot_z_pos = current_ee_pose.position.z

            robot_msg.leap_transX = transX
            robot_msg.leap_transY = transY
            robot_msg.leap_transZ = transZ
            

        processPub.publish(robot_msg)

    
    # FYDP Leap Subscriber
    rospy.Subscriber('leap_motion_coords', ControllerData, leapCallback, queue_size = 1)

    def robotEndpointStateCallback(data):
        global endpoint_effort_hist

        forces = data.wrench.force
        norm_total_force = np.sqrt(forces.x**2 + forces.y**2 + forces.z**2)
        robot_msg.joint_effort_norm = norm_total_force
        endpoint_effort_hist = np.append(endpoint_effort_hist, norm_total_force)
        if len(endpoint_effort_hist) > ENDPOINT_EFFORT_HIST_LEN:
            endpoint_effort_hist = np.delete(endpoint_effort_hist, 0)
        if all(x >= 30 for x in endpoint_effort_hist):
            robot_msg.movement_obstruction = True
        if all(x < 30 for x in endpoint_effort_hist):
            robot_msg.movement_obstruction = False

    rospy.Subscriber('robot/limb/right/endpoint_state',EndpointState, robotEndpointStateCallback, queue_size=1)
        


    print "######## Ready to move. #########"
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        # publish info on the publisher
        if not reset_robot_pose:
            message_array = 'x: ' + str(new_ee_pose.position.x) + ' y: ' + str(new_ee_pose.position.y) + ' z: ' + str(new_ee_pose.position.z) + ' r: ' + str(wrist_angle)
        else:
            message_array = 'reset pose x: ' + str(new_ee_pose.position.x) + ' y: ' + str(new_ee_pose.position.y) + ' z: ' + str(new_ee_pose.position.z) + ' r: ' + str(wrist_angle)
            reset_robot_pose = False
        pub.publish(message_array)
        
        #FYDP: Constantly print out pose 
        # current_ee_pose = get_current_ee_pose(sawyer_arm, fksvc)
        # if not current_ee_pose:
        #     rospy.logerr("Simple FK call FAILED")
        # else:
        #     print "Initial EE pose:\n" + str(current_ee_pose)
        
        


        # get the keyboard input
        c = intera_external_devices.getch()
        if c:
            # catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Terminating the program.")
            elif c in bindings:
                cmd = bindings[c]
                if c == '1':
                    new_ee_pose.position.x += delta
                    if not cmd[0](sawyer_arm, new_ee_pose, iksvc):
                        print "Impossible to move in x+ direction due to failed solution."
                        new_ee_pose.position.x -= delta
                elif c == 'q':
                    new_ee_pose.position.x -= delta
                    if not cmd[0](sawyer_arm, new_ee_pose, iksvc):
                        print "Impossible to move in x- direction due to failed solution."
                        new_ee_pose.position.x += delta
                if c == '2':
                    new_ee_pose.position.y += delta
                    if not cmd[0](sawyer_arm, new_ee_pose, iksvc):
                        print "Impossible to move in y_ direction due to failed solution."
                        new_ee_pose.position.y -= delta
                elif c == 'w':
                    new_ee_pose.position.y -= delta
                    if not cmd[0](sawyer_arm, new_ee_pose, iksvc):
                        print "Impossible to move in y- direction due to failed solution."
                        new_ee_pose.position.y += delta
                if c == '3':
                    new_ee_pose.position.z += delta
                    if not cmd[0](sawyer_arm, new_ee_pose, iksvc):
                        print "Impossible to move in z+ direction due to failed solution."
                        new_ee_pose.position.z -= delta
                elif c == 'e':
                    new_ee_pose.position.z -= delta
                    if not cmd[0](sawyer_arm, new_ee_pose, iksvc):
                        print "Impossible to move in z- direction due to failed solution."
                        new_ee_pose.position.z += delta
                if c == '4':
                    wrist_angle = wrist_angle + delta_wrist
                    if wrist_angle > 3.14 or wrist_angle < -3.14:
                        print "Wrist limit reached! "
                        wrist_angle = wrist_angle - delta_wrist
                    else:
                        cmd[0](sawyer_arm, wrist_angle)
                elif c == 'r':
                    wrist_angle = wrist_angle - delta_wrist
                    if wrist_angle > 3.14 or wrist_angle < -3.14:
                        print "Wrist limit reached! "
                        wrist_angle = wrist_angle + delta_wrist
                    else:
                        cmd[0](sawyer_arm, wrist_angle)
                elif c == 'x':
                    print "CAUTION: resetting robot pose."
                    cmd[0](sawyer_arm)
                    reset_robot_pose = True
                    # Get new EE pose
                    current_ee_pose = get_current_ee_pose(sawyer_arm, fksvc)
                    if not current_ee_pose:
                        rospy.logerr("Simple FK call FAILED")
                        done = True
                        rospy.signal_shutdown("Terminating the program.")
                    else:
                        print "New EE pose after reset:\n" + str(current_ee_pose)
                    # set the new pose to the current pose
                    new_ee_pose = current_ee_pose
                    print "######## Ready to move. #########"
                    print("Controlling joints. Press ? for help, Esc to quit.")

                # print "Before: " + str(new_ee_pose.position)
                # new_ee_pose = get_current_ee_pose(sawyer_arm, fksvc)
                # print "After: " + str(new_ee_pose.position)
                # if not new_ee_pose:
                #     rospy.logerr("Simple FK call FAILED, quitting.")
                #     done = True
                #     rospy.signal_shutdown("Terminating the program.")

                if c != 'x':
                    print cmd[1]
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print instructions

def sawyer_shutdown():
    print "Exiting the test program. Goodbye."





def main():
    global endpoint_effort_hist
    endpoint_effort_hist = np.zeros(ENDPOINT_EFFORT_HIST_LEN)

    """
    Use the keyboard to move the EE, position in x,y,z and rotation of the wrist (last joint)
    """
    robot_params = intera_interface.RobotParams()
    # get the name of the Sawyer arm to pass to intera sdk
    sawyer_arm_name = robot_params.get_limb_names()[0]

    global processPub
    processPub = rospy.Publisher('robot_data_message', RobotData, queue_size=10)

    # Initialize the ROS node
    print "Initializing node now..."
    rospy.init_node("sawyer_test_ee", anonymous = True)


    # for publisher
    rate = rospy.Rate(200) 

    # Initializee robot state
    print "Getting robot state now..."
    robot_state = intera_interface.RobotEnable(CHECK_VERSION)

    rospy.on_shutdown(sawyer_shutdown)

    # enable the robot
    print "Enabling robot now..."
    robot_state.enable()

    # call the function to move th EE
    move_ee(sawyer_arm_name, rate)


if __name__ == '__main__':
    main()
