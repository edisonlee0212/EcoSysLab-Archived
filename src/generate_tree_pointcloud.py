import pyecosyslab as pesl
import os

current_directory = os.getcwd()

os.mkdir(current_directory + "\\pc")

project_path = "C:\\Users\\lllll\\Desktop\\EcoSysLabProject\\test.eveproj"
target_descriptor_path = "C:\\Users\\lllll\\Desktop\\EcoSysLabProject\\TreeDescriptors\\Acacia.td"
target_tree_mesh_path = current_directory + "\\pc\\tree.obj"
target_tree_pointcloud_path = current_directory + "\\pc\\tree.ply"

pesl.start_project_windowless(project_path)
tmgs = pesl.TreeMeshGeneratorSettings()
pcps = pesl.PointCloudPointSettings()
pccs = pesl.PointCloudCaptureSettings()
##NOTE: The above code should only be run once for entire application lifespan. Do not start framework multiple times within single execution.

##NOTE: You may run below line multiple times for exporting multiple OBJs from multiple binvox inputs.
#Parameters: 
#1.		voxel grid radius
#2.		binvox absolute path
#3.		tree descriptor absolute path
#4.		growth delta time
#5.		growth iteration count

#6.		Tree mesh generator settings
#7.		enable tree mesh export
#8.		tree mesh output path

#9.		enable tree io
#10.	tree io output path
#11.	enable RBV
#12.	RBV output path
#13.	enable RBV mesh
#14.	RBV mesh output path
pesl.generate_tree_point_cloud(pcps, pccs, target_descriptor_path, 0.08220, 150, tmgs, target_tree_pointcloud_path, True, target_tree_mesh_path)