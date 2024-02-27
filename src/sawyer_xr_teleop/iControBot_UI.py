from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import QApplication, QMainWindow
import sys 
import fydp_mainscreen
from datetime import datetime


import rospy 
from std_msgs.msg import String
from sawyer_xr_teleop.msg import ControllerData, RobotData

from enum import Enum
'''
def window(): 
    app = QApplication(sys.argv)
    win = QMainWindow()
    win.setGeometry(0,0,3000,1000)
    win.setWindowTitle("iControbot")
    
    win.show()
    sys.exit(app.exec_())

window() 
'''
class ControllerErrorMessage(Enum): 
    GOOD = 0
    NOHAND = 1 
    TOOMANYHANDS = 2 
    DISCONNECTED = 3 

class RobotErrorMessage(Enum):
    NOSOLUTION = 1
    UNCAUGHTERROR = 2 
    DISCONNECTED = 3 

class RobotEdgeLimits(Enum): 
    YMAX = 1
    YMIN = 2
    XMIN = 3 
    XMAX = 4 
    ZMIN = 5
    ZMAX = 6


class UIApp(QMainWindow, fydp_mainscreen.Ui_MainWindow):
    # custom signal?
    goalSignal = QtCore.pyqtSignal()

    resetSignal = QtCore.pyqtSignal()

    stopSignal = QtCore.pyqtSignal()
    startSignal = QtCore.pyqtSignal()

    slowdownSignal = QtCore.pyqtSignal()

    collisionSignal = QtCore.pyqtSignal()

    leftEdgeSignal = QtCore.pyqtSignal()
    rightEdgeSignal = QtCore.pyqtSignal()
    topEdgeSignal = QtCore.pyqtSignal()
    bottomEdgeSignal = QtCore.pyqtSignal()
    zMiddleFrameEdgeSignal = QtCore.pyqtSignal()

    noHandsSignal = QtCore.pyqtSignal()
    tooManyHands = QtCore.pyqtSignal()
    cannotCommunicateCamera = QtCore.pyqtSignal()

    noSolutionFound = QtCore.pyqtSignal()
    unknownError = QtCore.pyqtSignal()
    cannotCommunicateSawyer = QtCore.pyqtSignal()

    global_leap_error = False
    global_stop = False
    goal_ticks = 0



    def __init__ (self, parent=None):
        super(UIApp,self).__init__(parent)
        rospy.init_node("sawyer_test_ee", anonymous = True)
        self.setupUi(self)
        self.setWindowTitle("iControbot")
        #self.setWindowOpacity(0.80)
        #self.z_middle_frame.setStyleSheet("background-color: rgba(0,0,0,0)")
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        #self.top_bar.setStyleSheet("background-color:white")
        #self.bottom_bar.setStyleSheet("background-color:white")
        #self.left_bar.setStyleSheet("background-color:white")
        #self.right_bar.setStyleSheet("background-color:white")
        #self.transparent_frame.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        #self.transparent_frame.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        #self.transparent_frame.setStyleSheet("background-color: transparent")
        rospy.Subscriber('leap_motion_coords', ControllerData, self.leapCallback)
        rospy.Subscriber('robot_data_message', RobotData, self.robotCallback)

        self.goalSignal.connect(self.setFinalPosGreen)
        self.resetSignal.connect(self.resetBars)
        self.leftEdgeSignal.connect(self.setLeftBarRed)
        self.rightEdgeSignal.connect(self.setRightBarRed)
        self.topEdgeSignal.connect(self.setTopBarRed)
        self.bottomEdgeSignal.connect(self.setBottomBarRed)
        self.zMiddleFrameEdgeSignal.connect(self.setZMiddleRed)
        self.slowdownSignal.connect(self.setMiddleYellow)
        self.collisionSignal.connect(self.setCollisionRed)

        self.noHandsSignal.connect(self.noHandsText)
        self.tooManyHands.connect(self.tooManyHandsText)
        self.cannotCommunicateCamera.connect(self.cannotCommunicateCameraText)

        self.noSolutionFound.connect(self.noSolutionFoundText)
        self.unknownError.connect(self.unknownErrorText)
        self.cannotCommunicateSawyer.connect(self.cannotCommunicateSawyerText)

        self.stopSignal.connect(self.setStop)
        self.startSignal.connect(self.setStart)





        # timer callback to reset bars in the case that no messages are received
        # message_window should be the max amount of time without receiving messages before resetting the bars
        #self.message_window = 0.2 # should be seconds
        #self.timer = rospy.Timer(self.message_window, self.timer_callback)
        #self.message_recieved_in_window = True
        #self.state_changed = False

        #self.timer = rospy.Timer(rospy.Duration(2), self.timer_callback)

    def noHandsText(self):
        current_time = datetime.now().strftime("%H:%M:%S")
        self.error_1.setText(current_time + "-ERROR: No hands detected")

    def tooManyHandsText(self):
        current_time = datetime.now().strftime("%H:%M:%S")
        self.error_1.setText(current_time + "-ERROR: Too many hands detected")

    def cannotCommunicateCameraText(self):
        current_time = datetime.now().strftime("%H:%M:%S")
        self.error_1.setText(current_time + "-ERROR: Cannot communicate with camera")

    def noSolutionFoundText(self):
        current_time = datetime.now().strftime("%H:%M:%S")
        self.error_1.setText(current_time + "-ERROR: No trajectory solution found")

    def unknownErrorText(self):
        current_time = datetime.now().strftime("%H:%M:%S")
        self.error_1.setText(current_time + "-ERROR: Unknown robot error")

    def cannotCommunicateSawyerText(self):
        current_time = datetime.now().strftime("%H:%M:%S")
        self.error_1.setText(current_time + "-ERROR: Cannot communicate with Sawyer")

    def setMiddleYellow(self):
        self.z_middle_frame.setStyleSheet("background-color: rgba(92,84,22,0.7)")
        current_time = datetime.now().strftime("%H:%M:%S")
        self.error_1.setText(current_time + "-WARN: Slow down")

    def setStop(self):
        self.z_middle_frame.setStyleSheet("background-color: rgba(255,0,0,0.7)")
        current_time = datetime.now().strftime("%H:%M:%S")
        self.error_1.setText(current_time + "- Stop Signal Received")

    def setStart(self):
        self.z_middle_frame.setStyleSheet("background-color: rgba(0,255,0,0.7)")
        current_time = datetime.now().strftime("%H:%M:%S")
        self.error_1.setText(current_time + "-Start Signal Received")

    def setCollisionRed(self): 
        self.top_bar.setStyleSheet("background-color:red")
        self.bottom_bar.setStyleSheet("background-color:red")
        self.left_bar.setStyleSheet("background-color:red")
        self.right_bar.setStyleSheet("background-color:red")
        self.z_middle_frame.setStyleSheet("background-color: rgba(0,0,0,0)")
        current_time = datetime.now().strftime("%H:%M:%S")
        self.error_1.setText(current_time + "-WARNING: Robot collision")
        
    def setFinalPosGreen(self):
        self.top_bar.setStyleSheet("background-color:green")
        self.bottom_bar.setStyleSheet("background-color:green")
        self.left_bar.setStyleSheet("background-color:green")
        self.right_bar.setStyleSheet("background-color:green")
        self.z_middle_frame.setStyleSheet("background-color: rgba(0,0,0,0)")

    def resetBars(self):
        self.top_bar.setStyleSheet("background-color:white")
        self.bottom_bar.setStyleSheet("background-color:white")
        self.left_bar.setStyleSheet("background-color:white")
        self.right_bar.setStyleSheet("background-color:white")
        self.z_middle_frame.setStyleSheet("background-color: rgba(0,0,0,0)")
    
    def setLeftBarRed(self):
        self.left_bar.setStyleSheet("background-color:red")


    def setRightBarRed(self):
        self.right_bar.setStyleSheet("background-color:red")


    def setTopBarRed(self):
        self.bottom_bar.setStyleSheet("background-color:red")


    def setBottomBarRed(self):
        self.top_bar.setStyleSheet("background-color:red")


    def setZMiddleRed(self):
        self.z_middle_frame.setStyleSheet("background-color: rgba(255,0,0,0.2)")

    #def reset_bars(self):
        #if self.state_changed:
     #   self.top_bar.setStyleSheet("background-color:white")
     #   self.bottom_bar.setStyleSheet("background-color:white")
     #   self.left_bar.setStyleSheet("background-color:white")
     #   self.right_bar.setStyleSheet("background-color:white")
     #   self.z_middle_frame.setStyleSheet("background-color: rgba(0,0,0,0)")
        #self.state_changed = False

    #def timer_callback(self,event):
    #    print("callback")
    #    # only run this if no messages have been received in the last X number of seconds 
    #    if not self.message_recieved_in_window:
    #        self.reset_bars()
    #    self.message_recieved_in_window = False

    def leapCallback(self, leap_data):
        #self.state_changed = True
        #self.reset_bars()
        #self.message_recieved_in_window = True
        if leap_data == None:
            return
        leap_error = leap_data.error_type
        print(leap_data.stop)
        
        if leap_data.stop:
            self.stopSignal.emit()
            self.global_stop = True
            return
        if self.global_stop and not leap_data.stop:
            self.startSignal.emit()
            self.global_stop = False
        
        if self.global_stop:
            return
    
        if leap_error == ControllerErrorMessage.NOHAND.value: 
            self.noHandsSignal.emit()
            self.global_leap_error = True
            
        elif leap_error == ControllerErrorMessage.TOOMANYHANDS.value: 
            self.tooManyHands.emit()
            self.global_leap_error = True

            
        elif leap_error == ControllerErrorMessage.DISCONNECTED.value: 
            self.cannotCommunicateCamera.emit()
            self.global_leap_error = True

        else:
            self.global_leap_error = False
          
        # elif leap_error == ControllerErrorMessage.GOOD.value:
           
           

    def robotCallback(self, data):
        #self.resetSignal.emit()
        if data == None: 
            return
        
        if self.global_stop:
            return
        robot_error = data.error_type
        robot_edge_limit = data.at_edge_limit
        #print(data)
        if data.movement_obstruction:
            self.collisionSignal.emit()
            return 
        
        if self.global_leap_error:
            self.resetSignal.emit()
            return
        

        if data.in_final_position: 
            # print("in final posio")
            self.goalSignal.emit()
            self.goal_ticks += 1
        else: 
            print("not final posit")
            self.resetSignal.emit()

        if data.slow_down:
            # slow down elseif here
            self.slowdownSignal.emit()

        if robot_edge_limit == RobotEdgeLimits.YMIN.value:
            # self.bottom_bar.setStyleSheet("background-color:red")
            self.bottomEdgeSignal.emit()
        elif robot_edge_limit == RobotEdgeLimits.YMAX.value:
            # self.top_bar.setStyleSheet("background-color:red")
            self.topEdgeSignal.emit()
        elif robot_edge_limit == RobotEdgeLimits.XMIN.value:
            # self.left_bar.setStyleSheet("background-color:red")
            self.leftEdgeSignal.emit()
        elif robot_edge_limit == RobotEdgeLimits.XMAX.value:
            # self.right_bar.setStyleSheet("background-color:red")
            self.rightEdgeSignal.emit()
        elif robot_edge_limit == RobotEdgeLimits.ZMAX.value:
            # self.z_middle_frame.setStyleSheet("background-color: rgba(255,0,0,0.2)")
            self.zMiddleFrameEdgeSignal.emit()
        elif robot_edge_limit == RobotEdgeLimits.ZMIN.value:
            # self.z_middle_frame.setStyleSheet("background-color: rgba(255,0,0,0.2)")
            self.zMiddleFrameEdgeSignal.emit()

        if robot_error == RobotErrorMessage.NOSOLUTION.value: 
            self.noSolutionFound.emit()
        elif robot_error == RobotErrorMessage.UNCAUGHTERROR.value: 
            self.unknownError.emit()
        elif robot_error == RobotErrorMessage.DISCONNECTED.value: 
            self.cannotCommunicateSawyer.emit()



def main():
    global message_window
    global timer 
    global message_recieved_in_window
    global state_changed

    message_window = 0.2 # should be seconds
    message_recieved_in_window = True
    state_changed = False

    app = QApplication(sys.argv)
    win = UIApp()
    
    win.show()
    app.exec_()

if __name__ == '__main__':
    main()
