import pyecosyslab as pesl
import os

current_directory = os.getcwd()

project_path = "C:\\Users\\lllll\\Desktop\\EcoSysLabProject\\test.eveproj"
binvox_path = "C:\\Users\\lllll\\Downloads\\binvox_files\\san_jose_acer_san_jose_9050_0_mesh.binvox"
target_descriptor_path = "C:\\Users\\lllll\\Desktop\\EcoSysLabProject\\TreeDescriptors\\Butter.td"
target_mesh_path = current_directory + "\\scd\\tree.obj"
target_tree_io_path = current_directory + "\\scd\\tree.treeio"
target_rbv_path = current_directory + "\\scd\\rbv.txt"
target_rbv_mesh_path = current_directory + "\\scd\\rbv.obj"
pesl.start_project_windowless(project_path)
tmgs = pesl.TreeMeshGeneratorSettings()

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
pesl.build_space_colonization_tree_data(2.0, binvox_path, target_descriptor_path, 0.08220, 250, tmgs, True, target_mesh_path, True, target_tree_io_path, True, target_rbv_path, True, target_rbv_mesh_path)
