#!/usr/bin/python

import rospy
from sensor_msgs.msg import Joy
from axis_camera.msg import Axis
from std_msgs.msg import Bool

class Teleop:
    def __init__(self):
        rospy.init_node('axis_ptz_speed_controller')
        self.initialiseVariables()
        self.pub = rospy.Publisher('cmd', Axis)
        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        self.pub_mirror = rospy.Publisher('mirror', Bool)
        
    def initialiseVariables(self):
        self.joy = None
        self.msg = Axis() # instantiate Axis message
        self.msg.autofocus = True # autofocus is on by default
        self.allowManualFocus = False # disable or enable manual focus
        self.mirror = False
        self.mirror_already_actioned = False # to stop mirror flip-flopping
        # sensitivities[0..5] corresponding to fwd, left, up, tilt right, 
        # tilt forwards, anticlockwise twist
        self.sensitivities = [120, -60, 40, 60, -40, 30]
        self.deadband = [0.2, 0.2, 0.2, 0.2, 0.2, 0.4]
       
    def spin(self):
        self.pub.publish(self.msg)
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.joy != None:
                self.createCmdMessage()
                self.createMirrorMessage()
            r.sleep()

    def createCmdMessage(self):
        '''Creates and publishes message to command the camera.  Spacenav axes
        are: [fwd, left, up, tilt_right, tilt_forward, twist_anticlockwise'''
        self.applyThresholds()
        self.msg.pan = self.axes_thresholded[3] * self.sensitivities[3]
        self.msg.tilt = self.axes_thresholded[4] * self.sensitivities[4]
        self.msg.zoom = self.axes_thresholded[0] * self.sensitivities[0]
        if self.allowManualFocus:
            self.checkForManualFocus()
        self.pub.publish(self.msg)

    def checkForManualFocus(self):
        '''Joystick button 0 sets autofocus to true, twisting the joystick sets
        autofocus to false and changes the focus.'''
        if self.joy.buttons[0]==1:
            self.msg.autofocus = True
        else:
            self.msg.focus = self.axes_thresholded[5] * self.sensitivities[5]
            if (abs(self.msg.focus)>0.00001):
                # Only turn autofocus off if msg.focus!=0
                self.msg.autofocus = False

    def applyThresholds(self):
        '''apply deadband to joystick output'''
        n = len(self.joy.axes)
        self.axes_thresholded = n * [0.0]
        for i in range(n):
            if (abs(self.joy.axes[i])>self.deadband[i]):
                self.axes_thresholded[i] = self.joy.axes[i]
                
    def joy_callback(self, data):
        self.joy = data

    def createMirrorMessage(self):
        '''Creates and publishes message to indicate image should be mirrored'''
        if self.joy.buttons[1]==1:
            if not self.mirror_already_actioned:
                self.mirror = not self.mirror
                self.mirror_already_actioned = True
        else:
            self.mirror_already_actioned = False
        self.pub_mirror.publish(Bool(self.mirror))
    
if __name__ == "__main__":
    teleop = Teleop()
    teleop.spin()
