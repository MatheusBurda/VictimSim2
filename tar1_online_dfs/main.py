import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
import time

## importa classes
from vs.environment import Env
from explorer import Explorer
from rescuer import Rescuer

def main(data_folder_name):
   
    # Set the path to config files and data files for the environment
    current_folder = os.path.abspath(os.getcwd())
    data_folder = os.path.abspath(os.path.join(current_folder, data_folder_name))

    
    # Instantiate the environment
    env = Env(data_folder)
    
    # Directions from AbstAgent.AC_INC
    # Performs a Up, Down, Right, Left, TopRight, BottomLeft, BottomRight, TopLeft
    direction_list = [0,4,2,6,1,5,3,7]
    N_AGENTS = 4

    # config files for the agents
    rescuer_file = os.path.join(data_folder, "rescuer_config.txt")
    explorer_file = os.path.join(data_folder, "explorer_config.txt")

    for i in range(N_AGENTS):
        # Instantiate agents rescuer and explorer
        resc = Rescuer(env, rescuer_file)
        # Explorer needs to know rescuer to send the map that's why rescuer is instatiated before
        exp = Explorer(env, explorer_file, resc, direction=direction_list[i % len(direction_list)])
    
    # Run the environment simulator
    env.run()
    
        
if __name__ == '__main__':
    """ To get data from a different folder than the default called data
    pass it by the argument line"""
    
    if len(sys.argv) > 1:
        data_folder_name = sys.argv[1]
    else:
        # data_folder_name = os.path.join("datasets", "data_10v_12x12")
        # data_folder_name = os.path.join("datasets", "data_42v_20x20")
        data_folder_name = os.path.join("datasets", "data_132v_100x80")
        
    main(data_folder_name)
