import cv2
import cv2.aruco as aruco
import numpy as np
import math
import socket
from time import strftime,localtime

camera_matrix =  [[1629.8388671875, 0.0, 949.8340035468573], [0.0, 1089.8487548828125, 503.28079681846066], [0.0, 0.0, 1.0]]
dist_matrix =  [[0.13116578339966167, -1.6157109375615122, 0.0020990123823193523, -0.0018148349228528386, 5.229738479798447]]

robot = '4'
waypoint1 = '6'
waypoint2 = 'right'
waypoint3 = '9'
correction = '7'
forward = 'w'.encode('utf-8')
smolright = 'd2'.encode('utf-8')
smolleft = 'a2'.encode('utf-8')
bigright = 'd1'.encode('utf-8')
bigleft = 'a1'.encode('utf-8')
stop = 'f'.encode('utf-8')
markers_found ={}
pixel_space ={}
Waypoints = {waypoint1:0,waypoint2:0,waypoint3:0}
aruco_actualperimeter = 520 #in mm
IP = "192.168.1.15" #"192.168.29.198"
UDP_PORT = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def findArucoMarker(img,markerSize=5,totalMarkers=250,draw=True):
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    key = getattr(aruco,f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids,rejected = aruco.detectMarkers(imgGray,arucoDict,parameters=arucoParam, cameraMatrix=np.array(camera_matrix), distCoeff=np.array(dist_matrix))
    #print(ids)
    if draw:
        aruco.drawDetectedMarkers(img,bboxs)
        if len(bboxs) != 0:
            int_bbox = np.int0(bboxs)
            # print(int_bbox)
            cv2.polylines(img, int_bbox, True, (0, 255, 0), 2)
    return (ids,bboxs)

def aruco_bbox(img,id,bbox):
    x_m = 0
    y_m = 0
    x_m = (bbox[0][0][0] + bbox[0][1][0] + bbox[0][2][0] + bbox[0][3][0])*0.25
    y_m = (bbox[0][0][1] + bbox[0][1][1] + bbox[0][2][1] + bbox[0][3][1])*0.25
    cv2.circle(img, (int(x_m), int(y_m)), 5, (0, 0, 255), -1)
    return [id,(x_m),(y_m)]

def length_bw_aruco(img,robotid,waypoint):
    x1,x2 = pixel_space[robotid][0],pixel_space[waypoint][0]
    y1,y2 = pixel_space[robotid][1],pixel_space[waypoint][1]
    distance = math.sqrt((x1-x2)**2+(y1-y2)**2)
    cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 1)
    xc = (x1+x2)/2
    yc = (y1+y2)/2
    return distance,int(xc),int(yc)
def move_to_waypoint(img,pixel_space,length_to_pixel_ratio,waypoint,threshold=160):
    if robot in pixel_space and waypoint in pixel_space:
        distance,xc,yc = length_bw_aruco(img,robot,waypoint)
        cntrl = distance*length_to_pixel_ratio
        print("cntrl",cntrl)
        if cntrl >= threshold:
            msg = forward
        else:
            msg = stop
            Waypoints[waypoint] = 1
    else:
        msg = stop
    return msg
# def correct(waypointa,waypointb):
#     msg = forward
#     try:
#         x1,y1 = (pixel_space[waypointa][0],pixel_space[waypointa][1])
#         x2,y2 = (pixel_space[waypointb][0], pixel_space[waypointb][1])
#         val = (y2-y1)*((pixel_space[robot][0]-x1)/(x2-x1)) - (pixel_space[robot][1]-y1)
#         if abs(val) >2000:
#             if val>0:
#                 msg = '3'.encode('utf-8')
#             else:
#                 msg = '2'.encode('utf-8')
#         else:
#             msg = forward
#     except KeyError:
#         print('lol')
#     return msg
def avg_top2corners(img,marker):
    xt = (marker[robot][0][0][0]+ marker[robot][0][1][0])*0.5
    yt = (marker[robot][0][0][1]+ marker[robot][0][1][1])*0.5
    cv2.circle(img, (int(xt), int(yt)), 5, (0, 0, 255), -1)
    return int(xt),int(yt)

def turn_angle(img,markers_found,waypoint):
    try:
        robot_heading = avg_top2corners(img, markers_found)
        p1x, p1y = pixel_space.get(robot)
        p2x, p2y = robot_heading
        p3x, p3y = pixel_space.get(waypoint)
        p12 = math.sqrt((p1x - p2x) ** 2 + (p1y - p2y) ** 2)
        p13 = math.sqrt((p1x - p3x) ** 2 + (p1y - p3y) ** 2)
        p23 = math.sqrt((p2x - p3x) ** 2 + (p2y - p3y) ** 2)
        angle = math.acos((p12 ** 2 + p13 ** 2 - p23 ** 2) / (2 * p12 * p13))
        angle = math.degrees(angle)
    except KeyError:
        print("...")
    return angle

def main():
    cap = cv2.VideoCapture(0)
    h_ini = int(strftime("%H", localtime()))
    m_ini = int(strftime("%M", localtime()))
    s_ini = int(strftime("%S", localtime()))

    while True:
        success,img = cap.read()
        ids,bboxs = findArucoMarker(img)
        if len(bboxs) != 0:
            for i in range(len(ids)):
                markers_found[str(ids[i][0])] = bboxs[i]
                marker = aruco_bbox(img,ids[i],markers_found[str(ids[i][0])])
                pixel_space[str(ids[i][0])] = (marker[1],marker[2])
                aruco_perimeter = cv2.arcLength(bboxs[0], True)
                length_to_pixel_ratio = aruco_actualperimeter/aruco_perimeter
                #val = correct(correction,waypoint1)
                
                if Waypoints.get(waypoint1) == 0:
                    msg = move_to_waypoint(img, pixel_space, length_to_pixel_ratio, waypoint1)

                elif Waypoints.get(waypoint1) == 1 and Waypoints.get(waypoint2) == 0:
                    angle = turn_angle(img, markers_found=markers_found, waypoint = waypoint2)
                    #Some angle stuff to left or right
                elif Waypoints.get(waypoint1) == 1 and Waypoints.get(waypoint2) == 0:
                    
                sock.sendto(msg, (IP, UDP_PORT))
        h = int(strftime("%H", localtime())) - h_ini
        m = int(strftime("%M", localtime())) - m_ini
        s = int(strftime("%S", localtime())) - s_ini
        if s < 0:
            s = s + 60
        ctime = str(h) + ":" + str(m) + ":" + str(s)
        cv2.putText(img, f'Time: {ctime}', (40, 70), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 3)
        cv2.imshow("Display", img)

        cv2.waitKey(1)
if __name__ == "__main__":
    main()


