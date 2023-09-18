import pyecosyslab as pesl
import os

current_directory = os.getcwd()

project_path = current_directory + "\\Temp\\Temp.eveproj"
target_yaml_path = "C:\\Users\\lllll\\Downloads\\RealWalnut_5001.yml"
target_mesh_path = current_directory + "\\out.obj"
target_capture_path = current_directory + "\\out.png"
pesl.start_project_windowless(project_path)
cgs = pesl.ConnectivityGraphSettings()
rs = pesl.ReconstructionSettings()
tmgs = pesl.TreeMeshGeneratorSettings()

pesl.yaml_to_mesh(target_yaml_path, cgs, rs, tmgs, target_mesh_path)
pesl.visualize_yaml(target_yaml_path, cgs, rs, tmgs, 0, 1.25, 2.5, 0, 0, 0, 4096, 4096, target_capture_path);
