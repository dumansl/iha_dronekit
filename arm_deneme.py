import rospy
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import *
import time
class ardupilot_controller:
    def __init__(self):
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        self.mode()
        time.sleep(2)
        self.arm()
        time.sleep(3)
    def mode(self):
        try:
            self.flightModeService(custom_mode='GUIDED')
            print("Guided moda gecildi...")
        except rospy.ServiceException, e:
            print "Guided moda gecilirken bir hata ile karsilasildi"%e
    def arm(self):
        try:
            self.armService(True)
            print("Arac arm edildi...")
        except rospy.ServiceException, e:
            print "Arm edilirken bir hata ile karsilasildi"%e
    def takeoff(self):
        try:
            self.takeoffService(altitude = 3)
            print("Arac takeoff ediliyor...")
        except rospy.ServiceException, e:
            print "Takeoff edilirken bir hata ile karsilasildi"%e

###################################################
if __name__ == '__main__':
    ardupilot_controller()
         