import numpy as np
import math3d as m3d
import cv2
import open3d

def generate_point_cloud(image_number):
    intrinsics = np.loadtxt("intrinsics/"+str(image_number)+".txt")
    extrinsics = np.loadtxt("extrinsics/"+str(image_number)+".txt", delimiter=", ")

    depth = np.load("depth/"+str(image_number)+".npy")
    rgb = cv2.imread("rgb/"+str(image_number)+".png")
    width = rgb.shape[0]
    height = rgb.shape[1]
    print(rgb.shape)

    ex_transform = m3d.Transform(extrinsics)
    ex_transform.invert()

    # Slower way but did this originally
    # point_cloud = np.zeros((rgb.shape[0],rgb.shape[1],3))
    # point_cloud[:,:,3:] = rgb
    # for i in range(rgb.shape[0]):
    #     for j in range(rgb.shape[1]):
    #         point_cloud[i][j][0] = depth[i][j]/intrinsics[0][0]*(i-intrinsics[0][2])
    #         point_cloud[i][j][1] = depth[i][j]/intrinsics[1][1]*(j-intrinsics[1][2])
    #         point_cloud[i][j][2] = depth[i][j]
    # Faster list comprehension. Used equations from: https://github.com/RobotLocomotion/drake/blob/master/perception/depth_image_to_point_cloud.cc
    # Also includes the extrinsic properties by multiplying by inverse of extrinsic transformation matrix
    point_cloud = np.array([[ex_transform*np.array([depth[i][j]/intrinsics[0][0]*(i-intrinsics[0][2]), depth[i][j]/intrinsics[1][1]*(j-intrinsics[1][2]), depth[i][j]]) for j in range(height)]for i in range(width)])

    # Use this for debugging and visualizing point cloud
    # pcd = open3d.PointCloud()
    # pcd.points = open3d.Vector3dVector(point_cloud.reshape(width*height,3))
    # open3d.draw_geometries([pcd])
    return point_cloud

pc_0 = generate_point_cloud(0)
pc_1 = generate_point_cloud(1)