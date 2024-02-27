import os, sys, inspect
src_dir = os.path.dirname(inspect.getfile(inspect.currentframe()))
arch_dir = '../lib/x64' if sys.maxsize > 2**32 else '../lib/x86'
sys.path.insert(0, os.path.abspath(os.path.join(src_dir, arch_dir)))

import numpy as np
import Leap

import rospy 
from std_msgs.msg import String
from sawyer_xr_teleop.msg import ControllerData

import scipy.signal
from scipy.interpolate import CubicSpline
import time


MIN_DELTA = 5

USE_INTERPOLATION = True

# FINGER_DELTA_STOP_THRESHOLD = 50

# [0,1] 0 being open palm
GRAB_STRENGTH_CLOSED_THRESH = 0.95


LEAP_X_MIN = -200
LEAP_X_MAX = 200
LEAP_Y_MIN = -200
LEAP_Y_MAX = 200
LEAP_Z_MIN = 50
LEAP_Z_MAX = 500

def checkWithinLimit(leap_x, leap_y, leap_z):
    if leap_x < LEAP_X_MAX and leap_x > LEAP_X_MIN and leap_y < LEAP_Y_MAX and leap_y > LEAP_Y_MIN and leap_z < LEAP_Z_MAX and leap_z > LEAP_Z_MIN:
        return True
    else: 
        return False

class SampleListener(Leap.Listener):
    # def __init__(self):
    #    super()._init_()
       

    global pub
    global disconnected 
    
    disconnected = False
    pub = rospy.Publisher('leap_motion_coords', ControllerData, queue_size=20)

    def on_connect(self, controller):
        print ("Connected")

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        #controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        #controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        disconnected = True        
        print ("Disconnected")
        hand_data = ControllerData()
        hand_data.error_type = 3
        pub.publish(hand_data)


    def on_frame(self, controller):
        global points_array
        global time_stamps
        global stop
        # global liveFilter

        frame = controller.frame()

        hand_data = ControllerData()
        hand_data.pos_x = 0 
        hand_data.pos_y = 0 
        hand_data.pos_z = 0 
        if (len(frame.hands) < 1): 
            hand_data.error_type = 1
            # pub.publish(hand_data)
            # return
        elif (len(frame.hands) > 1):
            hand_data.error_type = 2
            # pub.publish(hand_data)
            # return
        elif (disconnected): #DELETE THIS IF STATEMENT SINCE IF DISCCONECTED WILL NOT COME INTO ON_FRAME FUNCTION?
            hand_data.error_type = 3
            pub.publish(hand_data)
            rospy.loginfo("disonnected here passing data --" + hand_data)
            return
        else:
            hand_data.error_type = 0



        # Gesture Start
        for gesture in frame.gestures():
            if gesture.type == Leap.Gesture.TYPE_CIRCLE:
            
                for gest_pointable in gesture.pointables:
                    if gest_pointable.is_finger:
                        finger = Leap.Finger(gest_pointable)
                        finger_type = finger.type
                        if finger_type == 1: #1 is type index
                            hand_data.stop = False
                            stop = False
                            print("START")
                            pass 

        
        hands = frame.hands
        hand_position = hands[0].palm_position


        

        if not stop:
        
        # else:
            # hand_data.stop = False

            # normal = hands[0].palm_normal
            # roll_angle = normal.roll * Leap.RAD_TO_DEG
            
            # if abs(roll_angle) > 150: 
            #     hand_data.stop = True
            #     stop = True
            #     pub.publish(hand_data)
            #     return

# manual fist calculator
            # first_finger_tip_pos = hands[0].fingers[0].joint_position(3)
            # last_finger_tip_pos = hands[0].fingers[-1].joint_position(3)

            # finger_deltas = first_finger_tip_pos-last_finger_tip_pos

            # finger_deltas_norm = np.sqrt(finger_deltas[0]**2+finger_deltas[1]**2+finger_deltas[2]**2)

            # print(finger_deltas_norm)

            # if finger_deltas_norm < FINGER_DELTA_STOP_THRESHOLD:
            #     hand_data.stop = True
            #     stop = True

            #     points_array = []
            #     time_stamps = []
            #     pub.publish(hand_data)
            #     return
            
            # try Hand.grip_strength
            # print(hands[0].grab_strength)
            if hands[0].grab_strength >= GRAB_STRENGTH_CLOSED_THRESH:
                hand_data.stop = True
                stop = True
                print("FIST")
                points_array = []
                time_stamps = []
                pub.publish(hand_data)
                return
                
            
            # if np.sqrt((hand_position.x-prev_pos[0])**2 + (hand_position.y-prev_pos[1])**2 + (hand_position.z-prev_pos[2])**2) > MIN_DELTA:
            hand_data.pos_x = hand_position.x
            #Switch z and y to match Sawyer Frame of Reference
            hand_data.pos_y = hand_position.z
            hand_data.pos_z = hand_position.y
            # rospy.loginfo(hand_data)
            # pub.publish(hand_data)



            if USE_INTERPOLATION:
                # Spline Interpolation
                if len(points_array) < 5:
                    points_array.append([hand_position.x, hand_position.y, hand_position.z])
                    time_stamps.append(time.time())
                else:

                    # get time deltas
                    x = [time_stamp - time_stamps[0] for time_stamp in time_stamps]

                    # print(x)

                    cs = CubicSpline(x, points_array)

                    xs = np.arange(0, x[-1], 0.01)

                    interpolated_path = cs(xs)

                    # filtered_path = [liveFilter(point) for point in interpolated_path]

                    for via_point in interpolated_path:
                        hand_data.pos_x = via_point[0]
                        #Switch z and y to match Sawyer Frame of Reference
                        hand_data.pos_y = via_point[2]
                        hand_data.pos_z = via_point[1]
                        time.sleep(0.01)

                        # DO I NEED TO ADD A WAIT IN BETWEEN EACH MSG SEND OR CHANGE PUB QUEUE SIZE?

                        # rospy.loginfo(hand_data)
                        pub.publish(hand_data)
                        rospy.loginfo(hand_data)

                    points_array = []
                    time_stamps = []
            else:
                hand_data.pos_x = hand_position.x
                #Switch z and y to match Sawyer Frame of Reference
                hand_data.pos_y = hand_position.z
                hand_data.pos_z = hand_position.y
                # rospy.loginfo(hand_data)
                pub.publish(hand_data)
                
        # else:
        #     points_array = []
        #     time_stamps = []


def main():
    global points_array
    points_array = []
    global time_stamps
    time_stamps = []
    global stop 
    stop = True
    
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()
    controller.config.set("Gesture.Circle.MinRadius", 1000.0)
    controller.config.save

    rospy.init_node('leap_motion_node', anonymous=True)
    rate = rospy.Rate(200) #10 hz

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print ("Press Enter to quit...")
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)

if __name__ == "__main__":
    main()

