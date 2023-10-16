import pyecosyslab as pesl
import os

current_directory = os.getcwd()

project_path = "C:\\Users\\lllll\\Desktop\\EcoSysLabProject\\test.eveproj"
binvox_path = "C:\\Users\\lllll\\Downloads\\binvox_files\\san_jose_acer_san_jose_9050_0_mesh.binvox"
target_descriptor_path = "C:\\Users\\lllll\\Desktop\\EcoSysLabProject\\TreeDescriptors\\Butter.td"
target_mesh_path = current_directory + "\\out.obj"
pesl.start_project_windowless(project_path)
tmgs = pesl.TreeMeshGeneratorSettings()

#Parameters: voxel grid radius, binvox absolute path, tree descriptor absolute path, tmgs, output path, mesh generation settings, delta time, iteration
pesl.build_space_colonization_tree_obj(2.0, binvox_path, target_descriptor_path, target_mesh_path, tmgs, 0.08220, 250)
