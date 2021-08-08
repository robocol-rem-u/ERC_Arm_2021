#!/usr/bin/env python

import cv_bridge
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
# from std_msgs.msg import Bool

import cv2

def image_recived(msg):
    print("[INFO]: Image Received, showImage function called")
    # if MOVEMENT_RECIVED:
    showImage(cv_bridge.CvBridge().imgmsg_to_cv2(msg))
        # switch_movement_recived()

# def movement_recived(msg):
#     MOVEMENT_RECIVED = msg

# def switch_movement_recived():
#     MOVEMENT_RECIVED = False

def calculate_movement(frame):
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

    cX = 0
    cY = 0

    if len(corners) > 0:

        ids = ids.flatten()

        # for (markerCorner, markerID) in zip(corners, ids):
        markerCorner = corners[0]
        markerID = ids[0]

        print("[INFO]: ArUco {} detected".format(markerID))

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
        
    cv2.rectangle(frame, (MIN_CX, MIN_CY), (MAX_CX, MAX_CY), (255,0,0), 3)
    
    return ids, cX, cY,frame

def showImage(frame):

    print("[INFO]: Starting ArUco detection")

    ids, cX, cY, frame = calculate_movement(frame)

    if len(ids) > 0:
        a = open('markers.csv','w')
        for i in ids:
            if ids_reconocidos[i-1] == 0:
                a.write("{i} {cX} {cY} 0\n".format())
                ids_reconocidos[i-1] = 1
        a.close()

    cv2.imshow('image', frame)
    cv2.waitKey(10)

def main():
    rospy.init_node('calibracion_vision')

    global ids_reconocidos
    ids_reconocidos = [0,0,0,0,0,0,0,0,0]

    sub1 = rospy.Subscriber(IMAGE_TOPIC, Image, image_recived, queue_size=2)
    # sub2 = rospy.Subscriber(MOVEMENT_TOPIC, bool, movement_recived, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    
    # Aqui se definen las constantes utilizadas durante la ejecucion

    # El espacio entre MIN_CX y MAX_CX es el centro enrt

    MIN_CX = 190
    MAX_CX = 300
    MIN_CY = 210
    MAX_CY = 300
    
    IMAGE_TOPIC = '/camera_image/color/image_raw'
    # MOVEMENT_TOPIC = '/movement_detected'
    
    # MOVEMENT_RECIVED = True

    if cv_bridge and cv2:
        print("[INFO]: Libraries OK")

    print("[INFO]: Starting ArUco Params")

    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    arucoParams = cv2.aruco.DetectorParameters_create()

    print("[INFO]: Starting \'calibracion_vision\' node")

    main()