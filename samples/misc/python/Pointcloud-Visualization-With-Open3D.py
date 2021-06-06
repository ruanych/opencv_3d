import argparse
import open3d as o3d

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('-cloud', dest='cloud_path', help='Point Cloud file path', type=str)
    parser.add_argument('-label', dest='label_path', help='Point Cloud label file path', type=str, default=None)
    args = parser.parse_args()

    point_cloud_path = args.cloud_path
    label_path = args.label_path

    # read point cloud
    pcd1 = o3d.io.read_point_cloud(point_cloud_path)

    if label_path:
        # read label
        lables0 = open(label_path).read().split("\n")
        len1 = len(lables0)
        color0 = []

        #  Set the color of the point
        for i in range(len1):
            if lables0[i] == '0':
                color0.append([0.0, 0.0, 0.0])
            elif lables0[i] == '1':
                color0.append([0.259, 0.522, 0.957])
            elif lables0[i] == '2':
                color0.append([0.859, 0.267, 0.216])
            elif lables0[i] == '3':
                color0.append([0.959, 0.651, 0.0])
            elif lables0[i] == '4':
                color0.append([0.059, 0.616, 0.345])
            elif lables0[i] == '5':
                color0.append([0.4, 0.0, 0.8])
            elif lables0[i] == '6':
                color0.append([0.0, 1.0, 0.8])
            elif lables0[i] != '':
                color0.append([1.0, 1.0, 1.0])
        pcd1.colors = o3d.utility.Vector3dVector(color0)
    else:
        pcd1.paint_uniform_color([1, 0, 0])
    # Point cloud visualization
    o3d.visualization.draw_geometries([pcd1])
