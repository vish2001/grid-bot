import cv2
import numpy as np
import cv2.aruco as aruco
import math

marker_dimension = 0.13
worldx = 539    #889  #Values in millimeters
worldy = 922    #508

# modify these based on your enviornment settings
robot_ID = 4
bottom_left = 7  #31 this is the origin - positivex: towards bottom right - positivey: towards top left
bottom_right = 13
top_left = 6
top_right = 9
##########################################
def getMarkerCenter(corners):
 px = (corners[0][0] + corners[1][0] + corners[2][0]+ corners[3][0]) * 0.25
 py = (corners[0][1] + corners[1][1] + corners[2][1]+ corners[3][1]) * 0.25
 return [px,py]

def getMarkerRotation(corners):
 unit_x_axis = [1.,0.]
 center = getMarkerCenter(corners)
 right_edge_midpoint = (corners[1]+corners[2])/2.
 unit_vec = (right_edge_midpoint-center)/np.linalg.norm(right_edge_midpoint-center)
 angle = np.arccos(np.dot(unit_x_axis,unit_vec))
 return angle


#########################################3
try:
    camera_matrix =  [[1629.8388671875, 0.0, 949.8340035468573], [0.0, 1089.8487548828125, 503.28079681846066], [0.0, 0.0, 1.0]]
    dist_matrix =  [[0.13116578339966167, -1.6157109375615122, 0.0020990123823193523, -0.0018148349228528386, 5.229738479798447]]
    found_dict_pixel_space = {}
    found_dict_camera_space = {}
    found_dict_world_space = {}
    found_dict_homography_space = {}
    final_string = ""
    originRvec = np.array([0, 0, 1])
    markerRvec = np.array([0, 0, 0])
    # load aruco parameters
    parameters = cv2.aruco.DetectorParameters_create()
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
    cap = cv2.VideoCapture("0")
    while True:
        _,image = cap.read()
        #aruco-detection
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        data = aruco.detectMarkers(image,aruco_dict,parameters=parameters,cameraMatrix=camera_matrix,distCoeff=dist_matrix)
        corners = data[0]
        ids = data[1]
        originIDglobal = 0
        #print(corners,ids)
        if np.all(ids) != None:#if any markers were found
            result = aruco.estimatePoseSingleMarkers(corners,marker_dimension,cameraMatrix=camera_matrix,distCoeffs=dist_matrix)
            rvecs = result[0] #rotational vectors of markers
            tvecs = result[1] #translational vectors of markers
            #print(rvecs,tvecs)
            imgcpy = image

            #Setting bottom left as origin
            if bottom_left in ids:
                originID = np.where(ids == bottom_left)[0][0]
            else:
                originID = originIDglobal
            originCorners = corners[originID]
            originRvec = rvecs[originID]  # rotation vec of origin tag
            originTvec = tvecs[originID]  # translation vec of origin tag
            display = cv2.aruco.drawDetectedMarkers(imgcpy, corners, ids)  # display is image copy with boxes drawn around the tags


            for i in range(len(ids)):
                ID = ids[i]
                Rvecs = rvecs[i]
                Tvecs = tvecs[i]
                Corners = corners[i]
                display = cv2.aruco.drawAxis(imgcpy, camera_matrix, dist_matrix, Rvecs,Tvecs,0.05)  # draw 3d axis
                found_dict_pixel_space["" + str(ids[i][0])] = Corners  # put the corners of this tag in the dictionary
                print(found_dict_pixel_space)
                cv2.imshow('display', display)
            #Homography
            if (len(found_dict_pixel_space) >= 4):
                zero = found_dict_pixel_space[bottom_left][0][3]  # bottom left - 0
                x = found_dict_pixel_space[bottom_right][0][2]  # bottom right - 10
                y = found_dict_pixel_space[top_left][0][0]  # top left - 1
                xy = found_dict_pixel_space[top_right][0][1]  # top right - 11
            else:
                continue

            workspace_world_corners = np.array([[0.0, 0.0], [worldx, 0.0], [0.0, worldy], [worldx, worldy]], np.float32)  # 4 corners in millimeters
            workspace_pixel_corners = np.array([zero, x, y, xy], np.float32)  # 4 corners in pixels
            print(workspace_pixel_corners)
            # Homography Matrix
            h, status = cv2.findHomography(workspace_pixel_corners, workspace_world_corners)  # perspective matrix
            im_out = cv2.warpPerspective(image, h, (worldx, worldy))  # showing that it works
            for i in range(len(ids)):
                j = ids[i][0]
                corners_pix = found_dict_pixel_space[str(j)]
                corners_pix_transformed = cv2.perspectiveTransform(corners_pix, h)
                found_dict_homography_space[str(j)] = corners_pix_transformed
                #robot = found_dict_homography_space[str(robot_ID)][0]

            cv2.imshow('Warped Source Image', im_out)

        else:
            display = image
            cv2.imshow('display', display)
        # ###### sending the data to the robot ########################
        # robot_center = getMarkerCenter(robot)
        # robot_angle = getMarkerRotation(robot)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except Exception as e:
    print(e)
cap.release()
cv2.destroyAllWindows()