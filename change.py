import open3d as o3d
import sys
#Ensure the user supplies the name of the file to convert to a .pcd
if len(sys.argv) < 2:
    #If they don't, stop the program
    print("Error, please supply the .stl file name")
else:
    #If they do, grab the file name
    input_file_name = sys.argv[1]
    output_file_name = input_file_name[:-3] + "pcd"
    #Set the scale factor for the transformation to a point cloud
    # 0.05 for the Easter Island Statue
    # 0.00005 for x-35
    scale_factor = 0.05
    #Read the .stl file as a mesh
    mesh = o3d.io.read_triangle_mesh(input_file_name)
    #Convert it to a pcd with a certain number of points
    pcd = mesh.sample_points_uniformly(number_of_points=200000)
    #Shrink the model down
    for point in pcd.points:
        point *= scale_factor
    o3d.io.write_point_cloud(output_file_name, pcd)
