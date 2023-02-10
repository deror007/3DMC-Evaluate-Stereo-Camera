from geneticalgorithm import geneticalgorithm as ga
import sys
import ogl_viewer.viewer as gl
import pyzed.sl as sl
import numpy as np
import time 
import os 
import glob
import cv2
import open3d as o3d
import random
def getRGBAchannels(colour):

    #convert to binary

    int32bits = np.asarray(colour, dtype=np.float32).view(np.int32).item()  # item() optional
    binary = '{:032b}'.format(int32bits)
    #print(binary)

    # split binary into 4 bytes respective to each colour channel
    
    RGBA_bin = [binary[i:i+8] for i in range(0, len(binary), 8)]
    RGBA = [int(RGBA_bin[0],2),int(RGBA_bin[1],2),int(RGBA_bin[2],2),int(RGBA_bin[3],2)]
    # print(RGBA)
    return RGBA



def segment(file):

    image = cv2.imread(file) 

    r = cv2.selectROI("select the area", image)
                        # start_row: end_row     , start_column: end_column
    cropped_image = image[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]

    cv2.imshow("Cropped image", cropped_image)
    cv2.waitKey(0)
    
    roiCoords = [r[0],r[1],r[2],r[3]]

    return roiCoords

#System call Cloud Compare to fit best plane to point cloud
def fitPlaneCC(pcdDir):
    os.putenv("Cloud", pcdDir)
    os.system("fitPlane.bat")
    print("Fitted plane.")


def getRMS(test):

    file = open(r"D:\tests\{}_BEST_FIT_PLANE_INFO.txt".format(test))

    content = file.readlines()

    contentList = content[1].split(" ") #RMS found on 2nd line!
    
    #check if the fitting plane computation was carried out, if not except error.
    
    rms = float(contentList[-1])
    file.close()
    if(rms!=0):
        return rms
    else:
        return 1


##  Objective Function
def testStandardDeviation(X):

    depthModes = [sl.DEPTH_MODE.ULTRA, sl.DEPTH_MODE.QUALITY, sl.DEPTH_MODE.PERFORMANCE]
    camResolution =[sl.RESOLUTION.HD720, sl.RESOLUTION.HD1080, sl.RESOLUTION.HD2K]
    #sensingModes = [sl.SENSING_MODE.STANDARD, sl.SENSING_MODE.FILL]

    #set initial parameters
    dMode=depthModes[int(X[1])]
    cRes=camResolution[int(X[2])]

    #set runtime parameters
    conf = int(X[0])

    #NOTE: files that get re-opened again crash the system. Need to diversify the names!
    test= str(dMode)+"_"+str(cRes)+"_"+str(conf)+"#"+str(random.randint(0, 1000))

    print ("\nTEST: | "+str(dMode)+" | "+str(cRes)+" | "+str(conf)+" |\n")


    
    init = sl.InitParameters(depth_mode=dMode,   #dMode[0],
                            coordinate_units=sl.UNIT.METER,
                            coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)
    init.depth_minimum_distance = 0.1  # 30 cm
    init.camera_resolution=cRes
    init.depth_stabilization = True
    

    zed = sl.Camera()
    status = zed.open(init)    
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    res = sl.Resolution()
    res.width = 1920
    res.height = 1080

    #set point cloud
    point_cloud = sl.Mat(res.width, res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU) 

    #set runtime parameters

    rtParams = sl.RuntimeParameters(confidence_threshold = conf,
    texture_confidence_threshold = 100,
    measure3D_reference_frame = sl.REFERENCE_FRAME.CAMERA)
    rtParams.sensing_mode = sl.SENSING_MODE.STANDARD
    rtParams.enable_depth = True

    rtParams.remove_saturated_areas = False   #lets see what this does
 

    if zed.grab(rtParams) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA,sl.MEM.CPU, res)


        point_cloud.write(r"D:\tests\{}.pcd".format(test))   #"C:\Users\deror\OneDrive\Desktop\Zed2Project\Project\depth_sensing\python\tests\{}.pcd".format(test))

        #retrieve pointcloud with only roi!
        point3d_position = np.zeros((int(roi[2])*int(roi[3]),3))
        # point3d_color = np.zeros((int(roi[2])*int(roi[3]),3))   # colour
        pcd=o3d.geometry.PointCloud()
        ii = 0 
        for j in range(int(roi[1]),int(roi[1]+roi[3])):
            for i in range(int(roi[0]),int(roi[0]+roi[2])):
                # print(pc.get_value(x,y))
                b=point_cloud.get_value(i,j)[0]
                point3D = point_cloud.get_value(i,j)[1]
                if b.name=='SUCCESS':

                    point3d_position[ii][:]=np.array(point3D[0:3])

                ii+=1

        pcdPixelCount = int(roi[2])*int(roi[3]) #This is used potentially for objective function to be fairer as lower confidences and resolutions are closer to 0
        pcd.points = o3d.utility.Vector3dVector(point3d_position) 

        o3d.io.write_point_cloud(r"D:\tests\{}.pcd".format(test), pcd)



        #Fit plane to segmented point cloud
        fitPlaneCC(r"D:\tests\{}.pcd".format(test))

        rms=getRMS(test)
        print("RMSE:", rms)

        print("\nCompleted Test: | "+str(dMode)+" | "+str(cRes)+" | "+str(conf)+"\n")

        with open("gaSolution.txt", 'a') as file:
            file.write(str(dMode)+" "+str(cRes)+" "+str(conf)+": RMS= "+str(rms)+"\n")

    zed.close()

    #NEW OBJECTIVE FUNCTION = FITTED RMS / # of PIXELS IN POINT CLOUD   maybe

    return rms
    ##NOTE:IMPORTANT SOME INIT PARAMETERS DO NOT WORK!! E.G VGA CANNOT COMPUTE STANDARD DEVIATION!!! Something about manually setting octree to settle this.

def segmentRegion():
        # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.HD2K
    init_params.depth_stabilization = False
    #init_params.depth_minimum_distance = 2.3  # 30 cm

    #NOTE: depth stabilization is turned off to help zed 2 camera not have memory leaks when doing the genetic algorithm. 
    #This worsens performance of getting correct depth so 
    #this should ne taken into account with final results.
    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
    # Setting the depth confidence parameters
    runtime_parameters.confidence_threshold = 100
    runtime_parameters.textureness_confidence_threshold = 100

    res = sl.Resolution()
    res.width = 1920
    res.height = 1080


    image = sl.Mat()
    pc = sl.Mat(res.width, res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU) 
    
    # A new image is available if grab() returns SUCCESS
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:

        # Retrieve left image
        zed.retrieve_image(image, sl.VIEW.LEFT)
        image.write("segmentThis.png")

        global roi
        roi=segment("segmentThis.png")

    # Close the camera
    zed.close()


if __name__ == "__main__":

    segmentRegion()

    #Genetic Algorithm
    params ={'max_num_iteration': 20,\
                'population_size': 5,\
                'mutation_probability':0.6,\
                'elit_ratio': 0.2,\
                'crossover_probability': 0.3,\
                'parents_portion': 0.2,\
                'crossover_type':'uniform',\
                'max_iteration_without_improv': 10}

    #conf, depth mode, camera resolution
    varbound=np.array([[25,100],[0,2],[0,2]])
    vartype=np.array([['int'],['int'],['int']])
    model=ga(function=testStandardDeviation,dimension=3,variable_type_mixed=vartype,variable_boundaries=varbound, algorithm_parameters=params)
    print("Calculating...")

    model.run()

    convergence=model.report #Convergence model as dictionary, maybe useful for displaying results!
    solution=model.output_dict

    print(solution)

