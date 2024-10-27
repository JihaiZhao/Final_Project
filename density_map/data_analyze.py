import csv

# List of input files
input_files = ['s2.csv', 's3.csv', 's4.csv', 's5.csv', 's6.csv', 's7.csv', 's8.csv']
output_files = ['s2_after.csv', 's3_after.csv', 's4_after.csv', 's5_after.csv', 's6_after.csv', 's7_after.csv', 's8_after.csv']

# List of variables we want to extract
desired_variables = ['O_T_EE']

# Function to parse the Franka state data from the file
def parse_franka_state(filename):
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

# Iterate through the list of input files and process each one
for input_file, output_file in zip(input_files, output_files):
    # Parse the input file and extract data
    robot_data = parse_franka_state(input_file)

    # Save the extracted data to the output CSV file
    save_to_csv(robot_data, output_file)

    print(f"Data saved to {output_file}")
