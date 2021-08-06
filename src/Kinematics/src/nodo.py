#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
import time

def talker():
    
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/robocol/topico', String, queue_size=10)
    time.sleep(1)
    rate = rospy.Rate(10) # 10hz
    dire = String() 
    dire.data="star"#str(input("Ingrese direccion: ")) 
    pub.publish(dire)
    while not rospy.is_shutdown():
    	rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


