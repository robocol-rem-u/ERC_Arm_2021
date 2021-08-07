import cv2
import numpy as np

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
arucoParams = cv2.aruco.DetectorParameters_create()

MIN_CY = 160
MAX_CY = 320
MIN_CX = 210
MAX_CX = 427

cap = cv2.VideoCapture(0)
while cap.isOpened():
    _, frame = cap.read()

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
        elif cX > MAX_CX:
            print("Mover a la derecha")
        elif cY < MIN_CY:
            print("Mover arriba")
        elif cY > MAX_CY:
            print("Mover abajo")
        else:
            print("Perfecto")

        # print(f"[INFO] ArUco marker ID: {markerID}")

    cv2.rectangle(frame, (MIN_CX, MIN_CY), (MAX_CX, MAX_CY), (255,0,0), 3)

    cv2.imshow("Image", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break


