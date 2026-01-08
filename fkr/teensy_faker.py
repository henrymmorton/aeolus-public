import numpy as np
from datetime import datetime 
import time
import argparse
from core.tnsy_interface.teensy_state import TeensyState

class TeensyStateStore:

    def __init__(self, state_store_path, sim_now):
        """
        Loads a state store for teensy stat
        
        param state_store_path: The path to the teensy state file
        param sim_now: Whether to overwrite teensy timestamps wiht current time
        """
        state_store_numpy = np.loadtxt(state_store_path, delimiter=',')
        state_store_list = []

        self.sim_now = sim_now

        for state_instance in state_store_numpy:
            current_state = TeensyState()
            current_state.read_state_from_store(state_instance)
            state_store_list.append(current_state)

        self.state_store = state_store_list

    def get_state_from_state_store(self):
        """
        Gets a teensy state from the state store and then waits 1/100s for the next one

        param state_store: The teensy state store list
        """
        current_state = self.state_store.pop(0)

        # If we are doing a current sim, overwrite the timestamp with now
        if self.sim_now:
            current_state.timestamp = datetime.now()

        time.sleep(0.01)
        return current_state

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_file", type=str, help="Full file path for input")
    args = parser.parse_args()
    
    input_file = args.input_file
    
    state_store = spin_up_teensy_state_store(input_file)



    

    
        





