import csv
import matplotlib.pyplot as plt

# Function to parse the Franka state data from the file
def parse_franka_state(filename, desired_variables):
    robot_data = dict()

    # Open the input file and process each line
    with open(filename, 'r') as file:
        for line in file:
            # Check each desired variable in the line
            for var in desired_variables:
                index = line.find("\"" + var + "\"")
                
                if index > 0:
                    # Find the start and end of the data array for the variable
                    dat_start = line.find('[', index + 1)
                    dat_end = line.find(']', dat_start + 1)
                    dat = line[dat_start + 1:dat_end]

                    # Handle trailing commas
                    if dat[-1] == ',':
                        dat = dat[:-1]

                    # Convert the data to a list of floats, or handle if empty
                    if not dat:
                        num_data = []
                    else:
                        try:
                            num_data = [float(x) for x in dat.split(',')]
                        except ValueError:
                            num_data = dat

                    # Append the data to the dictionary
                    if var in robot_data:
                        robot_data[var].append(num_data)
                    else:
                        robot_data[var] = [num_data]

    return robot_data

# Function to save the extracted data to a CSV file
def save_to_csv(data, filename):
    with open(filename, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Write the header
        writer.writerow(['O_T_EE_12', 'O_T_EE_13'])
        
        # Write the O_T_EE[12] and O_T_EE[13] values
        for entry in data['O_T_EE']:
            if len(entry) > 13:
                writer.writerow([entry[12], entry[13]])

def compare_two_trajectory(desire_path, panda_path):
    # Initialize lists for x and y values for both trajectories
    x_desire = []
    y_desire = []
    x_panda = []
    y_panda = []

    # Read the desire_path CSV file
    with open(desire_path, mode='r') as file:
        reader = csv.reader(file)
        for row in reader:
            # Convert each value to float and append to respective lists
            x_desire.append(float(row[0]))
            y_desire.append(float(row[1]))

    # Read the panda_path CSV file, skipping the first 10 rows
    with open(panda_path, mode='r') as file:
        reader = csv.reader(file)
        for i, row in enumerate(reader):
            if i < 100:
                continue  # Skip the first 10 rows
            # Convert each value to float and append to respective lists
            x_panda.append(float(row[0]))
            y_panda.append(float(row[1]))
            
    print(len(x_desire))
    print(len(x_panda))

    # Plot the data for both trajectories
    plt.figure(figsize=(8, 6))
    plt.plot(x_desire, y_desire, marker='o', linestyle='-', color='b', label='Desired Trajectory')
    plt.plot(y_panda, x_panda, marker='x', linestyle='-', color='r', label='Panda Trajectory')
    
    plt.title('Comparison of Desired and Panda Trajectories')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    # # List of input files
    # input_files = ['s2.csv', 's3.csv', 's4.csv', 's5.csv', 's6.csv', 's7.csv', 's8.csv']
    # output_files = ['s2_after.csv', 's3_after.csv', 's4_after.csv', 's5_after.csv', 's6_after.csv', 's7_after.csv', 's8_after.csv']

    # # List of variables we want to extract
    # desired_variables = ['O_T_EE']

    # # Iterate through the list of input files and process each one
    # for input_file, output_file in zip(input_files, output_files):
    #     # Parse the input file and extract data
    #     robot_data = parse_franka_state(input_file, desired_variables)

    #     # Save the extracted data to the output CSV file
    #     save_to_csv(robot_data, output_file)

    #     print(f"Data saved to {output_file}")
        
    desire_path = 'trajectory_data.csv'
    panda_path = '/home/jihai/Jihai/Final_Project/franka_trajectory.csv'
    compare_two_trajectory(desire_path, panda_path)

main()