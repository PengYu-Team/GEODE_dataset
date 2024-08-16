import open3d as o3d
import numpy as np
import struct
import os

def read_bin_file(filename, typeLiDAR):
    points = []
    with open(filename, "rb") as file:
        while True:
            if typeLiDAR == "Velodyne":
                data = file.read(22)
                if len(data) < 22:
                    break
                x, y, z, intensity = struct.unpack('ffff', data[:16])
                ring = struct.unpack('H', data[16:18])
                time = struct.unpack('f', data[18:22])
            elif typeLiDAR == "Ouster":
                data = file.read(30)
                if len(data) < 26:
                    break
                x, y, z, intensity = struct.unpack('ffff', data[:16])
                t = struct.unpack('I', data[16:20]) 
                reflectivity, ring, ambient = struct.unpack('HHH', data[20:26])
                range = struct.unpack('I', data[26:30])
            elif typeLiDAR == "Livox":
                data = file.read(18)
                if len(data) < 18:
                    break
                x, y, z, intensity = struct.unpack('ffff', data[:16])
                tag, line = struct.unpack('BB', data[16:18])
            else:
                raise ValueError("Unsupported LiDAR type")
            points.append([x, y, z])
    return points

def visualize_point_cloud(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    o3d.visualization.draw_geometries([pcd])

# Example Usage
filename = input("Enter the path of bin file: ")
typeLiDAR = input("Enter the LiDAR type (Livox, Aeva, Ouster, Velodyne): ")  # Change as per your LiDAR type: "Velodyne", "Ouster", or "Livox"
pointcloud = read_bin_file(filename, typeLiDAR)
visualize_point_cloud(pointcloud)
