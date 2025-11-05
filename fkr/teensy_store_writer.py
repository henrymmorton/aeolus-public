import numpy as np
import random
import argparse
import os

def generate_data(num_rows, column_inputs, output_file):
    """
    Generates a text file with `num_rows` rows and 16 columns,
    where each column's values are controlled by `column_inputs`.
    
    Parameters:
        num_rows (int): Number of rows to generate.
        column_inputs (list of callables or numbers): A list of 16 elements where each element
                         is either a constant value or a function that generates values.
        output_file (str): Path to save the generated text file.
    """
    if len(column_inputs) != 16:
        raise ValueError("column_inputs must have exactly 16 elements")
    
    # Ensure the file exists before writing
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    data = np.zeros((num_rows, 16))
    
    for col in range(16):
        input_val = column_inputs[col]
        if callable(input_val):  # If it's a function, generate values dynamically
            data[:, col] = [input_val() for _ in range(num_rows)]
        else:  # If it's a constant, fill the column with that value
            data[:, col] = input_val
    
    np.savetxt(output_file, data, delimiter=',', fmt='%.6f')
    print(f"Data saved to {output_file}")

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_rows", type=int, help="Number of rows to generate")
    parser.add_argument("--output_file", type=str, help="Full file path for output")
    args = parser.parse_args()
    
    column_inputs = [0,0,0,0,0,0,0,0,0,0,0,0,100,4.5,4.5,0]
    
    generate_data(args.num_rows, column_inputs, args.output_file)