#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import bool
from cv_bridge import CvBridge

import cv2

def image_recived(msg):
    # if MOVEMENT_RECIVED:
    showImage(CvBridge().imgmsg_to_cv2(msg))
        # switch_movement_recived()

# def movement_recived(msg):
#     MOVEMENT_RECIVED = msg

# def switch_movement_recived():
#     MOVEMENT_RECIVED = False

def calculate_movement(frame):
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

    if len(corners) > 0:
        ids = ids.flatten()

        # for (markerCorner, markerID) in zip(corners, ids):
        markerCorner = corners[0]
        markerID = ids[0]
        corners = markerCorner.reshape((4,2))
        topLeft, topRight, bottomRight, bottomLeft = corners

        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))
        
        cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

        cX = int((topLeft[0] + bottomRight[0]) / 2)
        cY = int((topLeft[1] + bottomRight[1]) / 2)
        cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

        cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if cX < MIN_CX:
            print("Mover a la izquierda")
            ret = IZQUIERDA
        elif cX > MAX_CX:
            print("Mover a la derecha")
            ret = DERECHA
        elif cY < MIN_CY:
            print("Mover arriba")
            ret = ARRIBA
        elif cY > MAX_CY:
            print("Mover abajo")
            ret = ABAJO
        else:
            print("Perfecto")
            ret = ADELANTE
        
        cv2.rectangle(frame, (MIN_CX, MIN_CY), (MAX_CX, MAX_CY), (255,0,0), 3)
    
        return ret

def showImage(frame):

    msg = calculate_movement(frame)
    pub.publish(msg)

    cv2.imshow('image', frame)
    cv2.waitKey(10)

def main():
    rospy.init_node('calibracion_vision')

    global pub
    pub = rospy.Publisher('movimiento_ajuste', Float32, queue_size=2)

    sub1 = rospy.Subscriber(IMAGE_TOPIC, Image, image_recived, queue_size=2)
    # sub2 = rospy.Subscriber(MOVEMENT_TOPIC, bool, movement_recived, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    

    MIN_CX = 160
    MAX_CX = 320
    MIN_CY = 210
    MAX_CY = 427

    IZQUIERDA = 1
    DERECHA = 2
    ARRIBA = 3
    ABAJO = 4
    ADELANTE = 5
    
    IMAGE_TOPIC = '/camera_image/color/image_raw'
    # MOVEMENT_TOPIC = '/movement_detected'
    
    # MOVEMENT_RECIVED = True

    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    arucoParams = cv2.aruco.DetectorParameters_create()

    main()
