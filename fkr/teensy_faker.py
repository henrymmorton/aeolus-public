import numpy as np
import datetime 
import time
import argparse
from core.tnsy_interface.teensy_state import TeensyState

def spin_up_teensy_state_store(state_store_path):
    """
    Loads a state store for teensy state
    """
    state_store_numpy = np.loadtxt(state_store_path, delimiter=',')
    state_store_list = []

    for state_instance in state_store_numpy:
        current_state = TeensyState()
        current_state.read_state_from_store(state_instance)
        state_store_list.append(current_state)

    return state_store_list

def get_state_from_state_store(state_store):
    """
    Gets a teensy state from the state store and then waits 1/100s for the next one

    param state_store: The teensy state store list
    """
    current_state = state_store.pop(0)
    time.sleep(0.01)
    return current_state

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_file", type=str, help="Full file path for input")
    args = parser.parse_args()
    
    input_file = args.input_file
    
    state_store = spin_up_teensy_state_store(input_file)



    

    
        





