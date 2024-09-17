import pandas as pd

# Function to read a CSV file, transpose it, and save as another CSV file
def transpose_csv(input_file, output_file):
    # Read the CSV file
    df = pd.read_csv(input_file)
    
    # Transpose the DataFrame
    df_transposed = df.transpose()
    
    # Save the transposed DataFrame to a new CSV file
    df_transposed.to_csv(output_file, header=False, index=False)

# Specify input and output file paths
input_file = '/home/shobot/sim_ws/src/moveit_config/scripts/joint_angles_apf_prm.csv'
output_file = '/home/shobot/sim_ws/src/moveit_config/scripts/joint_angles_mpc.csv'

# Call the function to transpose the CSV file
transpose_csv(input_file, output_file)
