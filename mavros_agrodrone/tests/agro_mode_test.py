#!/usr/bin/env python

import rospy

from agrodrone.msg import CompanionMode
from agrodrone.srv import SetCompanionMode

 
msg = CompanionMode()
msg.mode= "Autospray"
msg.state = "TrackSpray"


def run():
    rospy.init_node("agro_mode_test")
    rate = rospy.Rate(1)
    mode_pub = rospy.Publisher("mavros/agro_mode", CompanionMode, queue_size=10) 

    set_mode_service = rospy.Service('set_companion_mode', SetCompanionMode, set_companion_mode)
   
    while not rospy.is_shutdown():
        mode_pub.publish(msg)
        rate.sleep()

def set_companion_mode(data):
    msg.mode=data.mode_to_set
    print("Received mode")
    return True

if __name__ == "__main__":
    run()


