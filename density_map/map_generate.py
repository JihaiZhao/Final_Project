import numpy as np 
import csv

np.set_printoptions(precision=4)

import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.rcParams['axes.linewidth'] = 3
mpl.rcParams['axes.titlesize'] = 20
mpl.rcParams['axes.labelsize'] = 20
mpl.rcParams['axes.titlepad'] = 8.0
mpl.rcParams['xtick.major.size'] = 6
mpl.rcParams['xtick.major.width'] = 3
mpl.rcParams['xtick.labelsize'] = 20
mpl.rcParams['ytick.major.size'] = 6
mpl.rcParams['ytick.major.width'] = 3
mpl.rcParams['ytick.labelsize'] = 20
mpl.rcParams['lines.markersize'] = 5
mpl.rcParams['legend.fontsize'] = 15

input_files = ['s2_after.csv', 's3_after.csv', 's4_after.csv', 's5_after.csv', 's6_after.csv', 's7_after.csv', 's8_after.csv']
# input_files = ['s2_after.csv']


# Function to read first and last row and filter data
def collect_data(filename, step=150):
    with open(filename, 'r') as file:
        reader = csv.reader(file)

        # Skip header
        header = next(reader)
        
        # Read all rows
        rows = list(reader)

        if len(rows) > 0:
            # Convert first and last row to floats for comparison
            first_row = [float(x) for x in rows[0]]  # Benchmark first row
            last_row = [float(x) for x in rows[-1]]  # Benchmark last row
            
            # Start collecting only when the row differs from the first row
            collect_data = False
            collected_rows = []
            row_counter = 0  # Row counter to collect every 200 rows

            for row in rows:
                # Convert current row to float for comparison
                current_row = [float(x) for x in row]

                # Start collecting data if the row differs from the first row
                if abs(current_row[0] - first_row[0]) > 0.005 and abs(current_row[1] - first_row[1]) > 0.005 and not collect_data:
                    collect_data = True
                
                if collect_data:
                    if row_counter % step == 0:
                        # Collect data only every `step` rows (default 200)
                        collected_rows.append(row)
                    row_counter += 1
                
                # Stop collecting data if the row matches the last row
                if abs(current_row[0] - last_row[0]) < 0.005 and abs(current_row[1] - last_row[1]) < 0.005:
                    break

            return collected_rows
        else:
            return []
        
# Process each file and collect data based on the benchmark rows
for input_file in input_files:
    filtered_data = collect_data(input_file, step=150)
    
    # Extract data for plotting
    x_values = [float(row[0]) for row in filtered_data]  # First column (O_T_EE_12)
    y_values = [float(row[1]) for row in filtered_data]  # Second column (O_T_EE_13)
    
    # Use scatter plot for better visibility
    plt.scatter(y_values, x_values, label=input_file, alpha=0.7)

# Set the axis limits
plt.xlim(-0.3, 0.3)
plt.ylim(0.6, 0.2)

# Set the grid with specific spacing
plt.xticks([-0.3 + i * 0.05 for i in range(13)])  # Creates ticks from -0.3 to 0.3
plt.yticks([i * 0.05 for i in range(4, 13)])  # Creates ticks from 0.25 to 0.8
plt.grid(True)

# Set aspect ratio to equal to maintain the proportions
plt.gca().set_aspect('auto')

# Add plot details
plt.xlabel('Y')
plt.ylabel('X')
plt.title('Collected Data from Multiple Files')

# Plot the extra point (example coordinates)
obstacle_x = 0.0  # X-coordinate of the extra point
obstacle_y = 0.3  # Y-coordinate of the extra point
plt.scatter(obstacle_x, obstacle_y, color='red', label='obstacle', s=100)  # s is the size of the point
plt.legend()  # Show legend with file names

# Show the plot
plt.show()



### We are going to use 10 coefficients per dimension --- so 100 index vectors in total

num_k_per_dim = 100
ks_dim1, ks_dim2 = np.meshgrid(
    np.arange(num_k_per_dim), np.arange(num_k_per_dim)
)
ks = np.array([ks_dim1.ravel(), ks_dim2.ravel()]).T  # this is the set of all index vectors
print('First 5 index vectors: ')
print(ks[:5, :])

# define a 1-by-1 2D search space
L_list = np.array([0.3, 0.2])  # boundaries for each dimension

# Discretize the search space into 100-by-100 mesh grids
grids_x, grids_y = np.meshgrid(
    np.linspace(-0.3, L_list[0], 100),
    np.linspace(0.6, L_list[1], 100)
)
grids = np.array([grids_x.ravel(), grids_y.ravel()]).T
dx = 1.0 / 999
dy = 1.0 / 999  # the resolution of the grids