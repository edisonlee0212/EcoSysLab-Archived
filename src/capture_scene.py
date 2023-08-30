import pyecosyslab as pesl
import os
current_directory = os.getcwd()

project_path = "C:\\Users\\lllll\\Documents\\GitHub\\EvoEngine\\Resources\\Example Projects\\Rendering\\Rendering.eveproj"
output_path = current_directory + "\\out.png"

pesl.start_project_windowless(project_path)
pesl.capture_active_scene(1920, 1080, output_path)
