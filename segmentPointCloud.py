import numpy as np
import cv2
import pyzed.sl as sl
import math
import sys
import open3d as o3d


def getRGBAchannels(colour):

    #convert to binary

    int32bits = np.asarray(colour, dtype=np.float32).view(np.int32).item()  # item() optional
    binary = '{:032b}'.format(int32bits)
    #print(binary)

    # split binary into 4 bytes respective to each colour channel
    
    RGBA = [binary[i:i+8] for i in range(0, len(binary), 8)]
    #print(RGBA)
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

#RGB image and point cloud from Zed 2

def main():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.HD720

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
    segmentedPC = sl.Mat(res.width, res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)

    # A new image is available if grab() returns SUCCESS
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:

        # Retrieve left image
        zed.retrieve_image(image, sl.VIEW.LEFT)
        image.write("segmentThis.png")
        # Retrieve colored point cloud. Point cloud is aligned on the left image.
        zed.retrieve_measure(pc, sl.MEASURE.XYZRGBA)
        pc.write("segmentThisPointCloud.pcd")

        roi=segment("segmentThis.png")

        #retrieve pointcloud with only roi!
        point3d_position = np.zeros((int(roi[2])+int(roi[3]),3))
        pcd=o3d.geometry.PointCloud()
        ii, jj = 0 , 0
        for j in range(int(roi[1]),int(roi[1]+roi[3])):
            for i in range(int(roi[0]),int(roi[0]+roi[2])):
                # print(pc.get_value(x,y))
                b=pc.get_value(i,j)[0]
                point3D = pc.get_value(i,j)[1]
                if b.name=='SUCCESS':
                    # x = point3D[0]
                    # y = point3D[1]
                    # z = point3D[2]
                    color = point3D[3]
                    point3d_position[jj][:]=np.array(point3D[0:3])
                    # pcd.points.append(np.array(point3D[0:3]))
                # print(x,y,z)
                # pcd.points.append(np.array([x,y,z]))
                ii+=1
            jj+=1

        print("Done")
    
    pcd.points = o3d.utility.Vector3dVector(point3d_position) 
    # pcd.colors = o3d.utility.Vector3dVector(colors[:, :3]) # needs to be converted to RGB
    # o3d.visualization.draw_geometries(pcd)
    o3d.io.write_point_cloud("crop_of_pcl.pcd", pcd)



    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()
