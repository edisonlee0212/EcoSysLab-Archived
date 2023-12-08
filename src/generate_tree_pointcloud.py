import pyecosyslab as pesl
import os

current_directory = os.getcwd()

if not os.path.isdir(current_directory + "\\pc"):
	os.mkdir(current_directory + "\\pc")

project_path = "C:\\Users\\lllll\\Desktop\\EcoSysLabProject\\test.eveproj"
target_descriptor_path = "C:\\Users\\lllll\\Desktop\\EcoSysLabProject\\TreeDescriptors\\Elm.td"
target_tree_mesh_path = current_directory + "\\pc\\tree.obj"
target_tree_pointcloud_path = current_directory + "\\pc\\tree.ply"
target_tree_junction_path = current_directory + "\\pc\\tree.yml"

pesl.start_project_windowless(project_path)
tmgs = pesl.TreeMeshGeneratorSettings()
pcps = pesl.PointCloudPointSettings()
pccs = pesl.PointCloudCaptureSettings()
##NOTE: The above code should only be run once for entire application lifespan. Do not start framework multiple times within single execution.

##NOTE: You may run below line multiple times for exporting multiple OBJs from multiple binvox inputs.
#Parameters: 
#1.		point cloud point settings
#2.		point cloud capture settings
#3.		tree descriptor absolute path
#4.		growth delta time (0.0822 years equal to 1 month)
#5.		growth iteration count (96 iterations of 1 month is 8 years, which gives you a old tree by default)

#6.		tree mesh generator settings
#7.		tree point cloud output path
#8.		enable tree mesh export
#9.		tree mesh output path
#10		enable junction export
#11		junction path
pesl.generate_tree_point_cloud(pcps, pccs, target_descriptor_path, 0.08220, 96, tmgs, target_tree_pointcloud_path, False, target_tree_mesh_path, True, target_tree_junction_path)