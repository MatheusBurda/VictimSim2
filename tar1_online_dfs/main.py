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
    
    N_AGENTS = 4

    # Directions from AbstAgent.AC_INC
    # Performs a TopRight, BottomLeft, BottomRight, TopLeft, Up, Down, Right, Left
    direction_list = [1,5,3,7,0,4,2,6]
    N_EXP = N_AGENTS
    N_RESC = N_AGENTS # Remove Captain

    # config files for the agents
    rescuer_file = os.path.join(data_folder, "rescuer_config.txt")
    explorer_file = os.path.join(data_folder, "explorer_config.txt")

    resc_captain = Rescuer(env, rescuer_file, captain=None)

    for _ in range(N_RESC-1):
        # Instantiate agents rescuer and explorer
        Rescuer(env, rescuer_file, captain=resc_captain)

    for i in range(N_EXP):
        # Explorer needs to know rescuer captain to send the map
        Explorer(env, explorer_file, resc_captain=resc_captain, direction=direction_list[i % len(direction_list)])
    
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
        # data_folder_name = os.path.join("datasets", "data_225v_100x80")
        
    main(data_folder_name)
