import matplotlib.pyplot as plt
import numpy as np
import pickle


def magnitudes(x,y,z):

    mags = []
    for ii in range(x.shape[0]):
        mag = np.sqrt(x[ii]*x[ii] + y[ii]*y[ii] + z[ii]*z[ii])
        mags.append(mag)
    return np.array(mags)

def find_spike(data, threshold=0.015):
    if data.ndim == 1:  # Check if the data is 1D
        s = np.where(abs(data) > threshold)[0][0]
        print(s)
        return s
    else:  # If data is 2D, keep the original indexing
        s1 = np.where(abs(data[:,0]) > threshold)[0][0]
        s2 = np.where(abs(data[:,1]) > threshold)[0][0]
        s3 = np.where(abs(data[:,2]) > threshold)[0][0]
        print(s1, s2, s3)
        return np.min([s1, s2, s3])

def alignment_indices(velocities, acceleration, forces):
    # print("velocity", velocities)
    
    franka_spike = find_spike(velocities,0.0065)
    force_spike = find_spike(forces,0.20)

    shift = force_spike - franka_spike

    if shift > 0:
        min_length = min(forces.shape[0]-shift, acceleration.shape[0])
        force_ind = [shift, min_length+shift]
        franka_ind = [0,min_length]
    else:
        min_length = min(acceleration.shape[0]+shift, forces.shape[0])
        franka_ind = [-shift, min_length-shift]
        force_ind = [0, min_length]
    
    return force_ind, franka_ind

def calulate_mc_coeef(velocity, acceleration, force, position):

    mat_av = np.array([acceleration, velocity]).T
    force = force.reshape(-1,1)

    # inv_mat = np.linalg.pinv(mat_av)
    # test_mc = inv_mat @ force
    
    test_mc, residuals, rank, s = np.linalg.lstsq(mat_av, force, rcond=None)

    return test_mc

##### Load all Conditions ######
all_data = dict()

file_base = '/home/jihai/Jihai/final_project_data'
    
# Load Impedance_force Data
with open(file_base + '/ft/impedance_force_x_ft.pkl', 'rb') as file:
    f1 = pickle.load(file)
    # print("Franka Data:", f1)  # Debugging line

with open(file_base + '/franka/impedance_force_x.pkl', 'rb') as file:
    frank1 = pickle.load(file)
    # print("Franka Data:", frank1)  # Debugging line

with open(file_base + '/ft/impedance_force_y_ft.pkl', 'rb') as file:
    f2 = pickle.load(file)

with open(file_base + '/franka/impedance_force_y.pkl', 'rb') as file:
    frank2 = pickle.load(file)
    
with open(file_base + '/ft/impedance_force_z_ft.pkl', 'rb') as file:
    f3 = pickle.load(file)

with open(file_base + '/franka/impedance_force_z.pkl', 'rb') as file:
    frank3 = pickle.load(file)

# Load Damping data
with open(file_base + '/ft/damping_x_ft.pkl', 'rb') as file:
    f4 = pickle.load(file)

with open(file_base + '/franka/damping_x.pkl', 'rb') as file:
    frank4 = pickle.load(file)

with open(file_base + '/ft/damping_y_ft.pkl', 'rb') as file:
    f5 = pickle.load(file)

with open(file_base + '/franka/damping_y.pkl', 'rb') as file:
    frank5 = pickle.load(file)
    
with open(file_base + '/ft/damping_z_ft.pkl', 'rb') as file:
    f6 = pickle.load(file)

with open(file_base + '/franka/damping_z.pkl', 'rb') as file:
    frank6 = pickle.load(file)
        
# Load Friction Comp Data
with open(file_base + '/ft/friction_compensation_x_ft.pkl', 'rb') as file:
    f7 = pickle.load(file)

with open(file_base + '/franka/friction_compensation_x.pkl', 'rb') as file:
    frank7 = pickle.load(file)

with open(file_base + '/ft/friction_compensation_y_ft.pkl', 'rb') as file:
    f8 = pickle.load(file)

with open(file_base + '/franka/friction_compensation_y.pkl', 'rb') as file:
    frank8 = pickle.load(file)
    
with open(file_base + '/ft/friction_compensation_z_ft.pkl', 'rb') as file:
    f9 = pickle.load(file)

with open(file_base + '/franka/friction_compensation_z.pkl', 'rb') as file:
    frank9 = pickle.load(file)
    
# Load White Light Data
with open(file_base + '/ft/white_light_x_ft.pkl', 'rb') as file:
    f10 = pickle.load(file)

with open(file_base + '/franka/white_light_x.pkl', 'rb') as file:
    frank10 = pickle.load(file)

with open(file_base + '/ft/white_light_y_ft.pkl', 'rb') as file:
    f11 = pickle.load(file)

with open(file_base + '/franka/white_light_y.pkl', 'rb') as file:
    frank11 = pickle.load(file)
    
with open(file_base + '/ft/white_light_z_ft.pkl', 'rb') as file:
    f12 = pickle.load(file)

with open(file_base + '/franka/white_light_z.pkl', 'rb') as file:
    frank12 = pickle.load(file)
    
    
# Create a dictionary to store only the X-axis data for the comparisons
x_data = dict(
    impedance_force_x=dict(franka=frank1, ft=f1), 
    damping_x = dict(franka=frank4, ft=f4),
    friction_compensation_x=dict(franka=frank7, ft=f7), 
    white_light_x=dict(franka=frank10, ft=f10)
)

# Create a figure with 4 subplots for the X-axis comparison
fig, axs = plt.subplots(1, 4, figsize=(20, 5))
fig.suptitle('X-Axis Data Comparison')

# Loop over the conditions (impedance, friction compensation, white light)
names = list(x_data.keys())
for ii in range(len(names)):
    print(f'Condition = {names[ii]}')

    ft_data = x_data[names[ii]]['ft']
    franka_data = x_data[names[ii]]['franka']

    # Extract the X-axis force data
    forces_x = np.array(ft_data['f_x_norm'])
    # Convert the list to a NumPy array before slicing
    vel_x = np.array(franka_data['Cartesian_EE_Velocity_Jac'])[:, 0]
    accel_x = np.array(franka_data['Cartesian_EE_Acceleration_Jac'])[:, 0] 
    velocity_x = np.array(franka_data['Cartesian_EE_Velocity_Jac'])[:, 0]  # Assuming the velocity data is stored this way
    
    force_idx, franka_idx = alignment_indices(vel_x, accel_x, forces_x)

    aligned_forces = forces_x[force_idx[0]:force_idx[1]]  
    aligned_vel = vel_x[franka_idx[0]:franka_idx[1]]  

    # Plot X-force vs X-velocity in each subplot
    axs[ii].scatter(aligned_vel, aligned_forces, s=5, alpha=0.5)
    axs[ii].set_title(names[ii])
    axs[ii].set_xlim([-0.5, 0.5])
    axs[ii].set_ylim([-20.0, 20.0])
    axs[ii].set_ylabel('Force (N)')
    axs[ii].set_xlabel('Velocity (m/s)')

# Create a dictionary to store only the X-axis data for the comparisons
y_data = dict(
    impedance_force_y=dict(franka=frank2, ft=f2), 
    damping_y = dict(franka=frank5, ft=f5),
    friction_compensation_y = dict(franka=frank8, ft=f8), 
    white_light_y = dict(franka=frank11, ft=f11)
)

# Create a figure with 4 subplots for the X-axis comparison
fig, axs = plt.subplots(1, 4, figsize=(20, 5))
fig.suptitle('Y-Axis Data Comparison')

# Loop over the conditions (impedance, friction compensation, white light)
names = list(y_data.keys())
for ii in range(len(names)):
    print(f'Condition = {names[ii]}')

    ft_data = y_data[names[ii]]['ft']
    franka_data = y_data[names[ii]]['franka']

    # Extract the X-axis force data
    forces_y = np.array(ft_data['f_y_norm'])
    # Convert the list to a NumPy array before slicing
    vel_y = np.array(franka_data['Cartesian_EE_Velocity_Jac'])[:, 1]
    accel_y = np.array(franka_data['Cartesian_EE_Acceleration_Jac'])[:, 1] 
    velocity_y = np.array(franka_data['Cartesian_EE_Velocity_Jac'])[:, 1]  # Assuming the velocity data is stored this way
    
    force_idx, franka_idx = alignment_indices(vel_y, accel_y, forces_y)

    aligned_forces = forces_y[force_idx[0]:force_idx[1]]  
    aligned_vel = vel_y[franka_idx[0]:franka_idx[1]]  

    # Plot X-force vs X-velocity in each subplot
    axs[ii].scatter(aligned_vel, aligned_forces, s=5, alpha=0.5)
    axs[ii].set_title(names[ii])
    axs[ii].set_xlim([-0.5, 0.5])
    axs[ii].set_ylim([-20.0, 20.0])
    axs[ii].set_ylabel('Force (N)')
    axs[ii].set_xlabel('Velocity (m/s)')

# Create a dictionary to store only the X-axis data for the comparisons
z_data = dict(
    impedance_force_z = dict(franka=frank3, ft=f3), 
    damping_z = dict(franka=frank6, ft=f6),
    friction_compensation_z = dict(franka=frank9, ft=f9), 
    white_light_z = dict(franka=frank12, ft=f12)
)

# Create a figure with 4 subplots for the X-axis comparison
fig, axs = plt.subplots(1, 4, figsize=(20, 5))
fig.suptitle('Z-Axis Data Comparison')

# Loop over the conditions (impedance, friction compensation, white light)
names = list(z_data.keys())
for ii in range(len(names)):
    print(f'Condition = {names[ii]}')

    ft_data = z_data[names[ii]]['ft']
    franka_data = z_data[names[ii]]['franka']

    # Extract the X-axis force data
    forces_z = np.array(ft_data['f_z_norm'])
    # Convert the list to a NumPy array before slicing
    vel_z = np.array(franka_data['Cartesian_EE_Velocity_Jac'])[:, 2]
    accel_z = np.array(franka_data['Cartesian_EE_Acceleration_Jac'])[:, 2] 
    velocity_z = np.array(franka_data['Cartesian_EE_Velocity_Jac'])[:, 2] 
    
    force_idx, franka_idx = alignment_indices(vel_z, accel_z, forces_z)

    aligned_forces = forces_z[force_idx[0]:force_idx[1]]  
    aligned_vel = vel_z[franka_idx[0]:franka_idx[1]]  

    # Plot X-force vs X-velocity in each subplot
    axs[ii].scatter(aligned_vel, aligned_forces, s=5, alpha=0.5)
    axs[ii].set_title(names[ii])
    axs[ii].set_xlim([-0.5, 0.5])
    axs[ii].set_ylim([-20.0, 20.0])
    axs[ii].set_ylabel('Force (N)')
    axs[ii].set_xlabel('Velocity (m/s)')

# Show the plot
plt.tight_layout()
plt.show()







