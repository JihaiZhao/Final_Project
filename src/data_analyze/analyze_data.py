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
    if data.size == 0 or data.shape[1] < 3:
        raise ValueError("Input data must be non-empty and have at least 3 columns.")

    s1 = np.where(abs(data[:,0]) > threshold)[0][0]
    s2 = np.where(abs(data[:,1]) > threshold)[0][0]
    s3 = np.where(abs(data[:,2]) > threshold)[0][0]

    # Check if any spike was found for each signal
    if s1.size == 0 or s2.size == 0 or s3.size == 0:
        raise ValueError("No spikes found in the input data.")

    return np.min([s1,s2,s3])

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

trials = [1]
for ii in trials:
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
        
        
    all_data[ii] = dict(impedance_force_x=dict(franka=frank1, ft=f1), impedance_force_y=dict(franka=frank2, ft=f2), impedance_force_z=dict(franka=frank3, ft=f3),
                        damping_x=dict(franka=frank4, ft=f4), damping_y=dict(franka=frank5, ft=f5), damping_z=dict(franka=frank6, ft=f6),
                        friction_compensation_x=dict(franka=frank7, ft=f7), friction_compensation_y=dict(franka=frank8, ft=f8), friction_compensation_z=dict(franka=frank9, ft=f9),
                        white_light_x=dict(franka=frank10, ft=f10), white_light_y=dict(franka=frank11, ft=f11), white_light_z=dict(franka=frank12, ft=f12)
                        )
    # all_data[ii] = dict(damping=dict(franka=frank1, ft=f1))


show_3d = True
show_2d = False
trials = list(all_data.keys())
conditions = list(all_data[1].keys())
print(conditions)
# mode = 'Damping'

##### Loop over all conditions #######
if show_2d:
    # initialize plots
    # Velocity vs Force Plots
    fig1, ax1 = plt.subplots()
    fig1.suptitle('X Axis')
    fig2, ax2 = plt.subplots()
    fig2.suptitle('Y Axis')
    fig3, ax3 = plt.subplots()
    fig3.suptitle('Z Axis')
    fig4, ax4 = plt.subplots()
    fig4.suptitle('Magnitude')

    # Force and Velocity Plots
    fig6, ax6 = plt.subplots()
    fig6.suptitle('X Axis')
    fig7, ax7 = plt.subplots()
    fig7.suptitle('Y Axis')
    fig8, ax8 = plt.subplots()
    fig8.suptitle('Z Axis')
    fig9, ax9 = plt.subplots()
    fig9.suptitle('Magnitude')

mc_x_total = []
mc_y_total = []
mc_z_total = []

for mode in conditions:
    for ii in range(len(trials)):

        print(f'{mode} Condition, Trial {trials[ii]}')

        franka_data = all_data[trials[ii]][mode]['franka']
        ft_data = all_data[trials[ii]][mode]['ft']

        pos = np.array(franka_data['Cartesian_EE_Position_Diff'])
        vel = np.array(franka_data['Cartesian_EE_Velocity_Jac'])
        accel = np.array(franka_data['Cartesian_EE_Acceleration_Jac'])
        forces = np.array([ft_data['f_x_norm'], ft_data['f_y_norm'], ft_data['f_z_norm']]).T

        force_idx, franka_idx = alignment_indices(vel, accel, forces)

        aligned_forces = forces[force_idx[0]:force_idx[1], :]
        aligned_pos = pos[franka_idx[0]:franka_idx[1], :]
        aligned_vel = vel[franka_idx[0]:franka_idx[1], :]
        aligned_accel = accel[franka_idx[0]:franka_idx[1], :]

        # f = m*a - c*v
        # f = [a -v][[m] [c]]
        # inv([a -v])*[f] = [[m] [c]]

        # x
        # mat_av_x = np.array([aligned_accel[:,0], -1*aligned_vel[:,0]]).T
        # force_x = aligned_forces[:,0].reshape(-1,1)
        # inv_mat = np.linalg.pinv(mat_av_x)
        # test_mc = inv_mat @ force_x
        # print(test_mc)

        mc_x = calulate_mc_coeef(aligned_vel[:,0], aligned_accel[:,0], aligned_forces[:,0], aligned_pos[:,0])
        mc_y = calulate_mc_coeef(aligned_vel[:,1], aligned_accel[:,1], aligned_forces[:,1], aligned_pos[:,1])
        mc_z = calulate_mc_coeef(aligned_vel[:,2], aligned_accel[:,2], aligned_forces[:,2], aligned_pos[:,2])
        
        mc_x_total.append(mc_x)
        mc_y_total.append(mc_y)
        mc_z_total.append(mc_z)

        print(f'X: m = {mc_x[0]}, c = {mc_x[1]}')
        print(f'Y: m = {mc_y[0]}, c = {mc_y[1]}')
        print(f'Z: m = {mc_z[0]}, c = {mc_z[1]}')

        force_mag = magnitudes(aligned_forces[:,0], aligned_forces[:,1], aligned_forces[:,2])
        vel_mag = magnitudes(aligned_vel[:,0], aligned_vel[:,1], aligned_vel[:,2])
        accel_mag = magnitudes(aligned_accel[:,0], aligned_accel[:,1], aligned_accel[:,2])


        if show_2d:
            # X
            ax1.scatter(aligned_vel[:,0], aligned_forces[:,0], s=5, alpha=0.5)
            ax1.set_xlim([-0.4,0.4])
            ax1.set_ylim([-20.0,20.0])
            ax1.set_ylabel('Force (N)')
            ax1.set_xlabel('Velocity')
            ax1.set_title(f'Trial {trials[ii]}')
            ax1.legend(['Force vs Velocity'])

            # Y
            ax2.scatter(aligned_vel[:,1], aligned_forces[:,1], s=5, alpha=0.5)
            ax2.set_xlim([-0.4,0.4])
            ax2.set_ylim([-20.0,20.0])
            ax2.set_ylabel('Force (N)')
            ax2.set_xlabel('Velocity')
            ax2.set_title(f'Trial {trials[ii]}')
            ax2.legend(['Force vs Velocity'])

            # Z
            ax3.scatter(aligned_vel[:,2], aligned_forces[:,2], s=5, alpha=0.5)
            ax3.set_xlim([-0.4,0.4])
            ax3.set_ylim([-20.0,20.0])
            ax3.set_ylabel('Force (N)')
            ax3.set_xlabel('Velocity')
            ax3.set_title(f'Trial {trials[ii]}')
            ax3.legend(['Force vs Velocity'])

            # Magnitude
            ax4.scatter(vel_mag, force_mag, s=5, alpha=0.5)
            ax4.set_xlim([-0.025,0.4])
            ax4.set_ylim([-1.0,25.0])
            ax4.set_ylabel('Force (N)')
            ax4.set_xlabel('Velocity')
            ax4.set_title(f'Trial {trials[ii]}')
            ax4.legend(['Force Magnitude vs Velocity Magnitude'])

            # Force and Velocity Time Series
            # X Axis
            line1, = ax6.plot(aligned_vel[:,0], color='b')
            ax6.set_ylabel('Velocity', color='k')
            ax10 = ax6.twinx()
            line2, = ax10.plot(aligned_forces[:,0], color='r')
            ax10.set_ylabel('Force', color='k')
            ax6.legend([line1, line2], ['Velocity', 'Force'], loc='upper right')
            ax6.set_title(f'Trial {trials[ii]}')

            # Y Axis
            line1, = ax7.plot(aligned_vel[:,1], color='b')
            ax7.set_ylabel('Velocity', color='k')
            ax11 = ax7.twinx()
            line2, = ax11.plot(aligned_forces[:,1], color='r')
            ax11.set_ylabel('Force', color='k')
            ax7.legend([line1, line2], ['Velocity', 'Force'], loc='upper right')
            ax7.set_title(f'Trial {trials[ii]}')

            # Z Axis
            line1, = ax8.plot(aligned_vel[:,2], color='b')
            ax8.set_ylabel('Velocity', color='k')
            ax12 = ax8.twinx()
            line2, = ax12.plot(aligned_forces[:,2], color='r')
            ax12.set_ylabel('Force', color='k')
            ax8.legend([line1, line2], ['Velocity', 'Force'], loc='upper right')
            ax8.set_title(f'Trial {trials[ii]}')

            # Magnitude
            line1, = ax9.plot(vel_mag, color='b')
            ax9.set_ylabel('Velocity', color='k')
            ax13 = ax9.twinx()
            line2, = ax13.plot(force_mag, color='r')
            ax13.set_ylabel('Force', color='k')
            ax9.legend([line1, line2], ['Velocity', 'Force'], loc='upper right')
            ax9.set_title(f'Trial {trials[ii]}')
            
        if ii == 2:
            ax1[ii].set_xlabel('Velocity')
            ax2[ii].set_xlabel('Velocity')
            ax3[ii].set_xlabel('Velocity')
            ax4[ii].set_xlabel('Velocity')
            ax6[ii].set_xlabel('Time (ms)')
            ax7[ii].set_xlabel('Time (ms)')
            ax8[ii].set_xlabel('Time (ms)')
            ax9[ii].set_xlabel('Time (ms)')


    if show_3d:

        fig_name = 'Condition ' + str(mode) + ' X axis'
        fig14 = plt.figure(fig_name)
        ax14 = fig14.add_subplot(111, projection='3d')
        scatter = ax14.scatter(aligned_vel[::10,0], aligned_accel[::10,0], aligned_forces[::10,0], alpha=0.6)
        ax14.set_xlabel('Velocity')
        ax14.set_ylabel('Acceleration')
        ax14.set_zlabel('Force')
        ax14.set_xlim([-0.4,0.4])
        ax14.set_ylim([-0.4,0.4])
        ax14.set_zlim([-25.0,25.0])
        ax14.set_title('X axis')

        fig_name = 'Condition ' + str(mode) + ' Y axis'
        fig15 = plt.figure(fig_name)
        ax15 = fig15.add_subplot(111, projection='3d')
        scatter = ax15.scatter(aligned_vel[::10,1], aligned_accel[::10,1], aligned_forces[::10,1], alpha=0.6)
        ax15.set_xlabel('Velocity')
        ax15.set_ylabel('Acceleration')
        ax15.set_zlabel('Force')
        ax15.set_xlim([-0.4,0.4])
        ax15.set_ylim([-0.4,0.4])
        ax15.set_zlim([-25.0,25.0])
        ax15.set_title('Y axis')

        fig_name = 'Condition ' + str(mode) + ' Z axis'
        fig16 = plt.figure(fig_name)
        ax16 = fig16.add_subplot(111, projection='3d')
        scatter = ax16.scatter(aligned_vel[::10,2], aligned_accel[::10,2], aligned_forces[::10,2], alpha=0.6)
        ax16.set_xlabel('Velocity')
        ax16.set_ylabel('Acceleration')
        ax16.set_zlabel('Force')
        ax16.set_xlim([-0.4,0.4])
        ax16.set_ylim([-0.4,0.4])
        ax16.set_zlim([-25.0,25.0])
        ax16.set_title('Z axis')

        fig_name = 'Condition ' + str(mode) + ' Magnitude'
        fig17 = plt.figure(fig_name)
        ax17 = fig17.add_subplot(111, projection='3d')
        scatter = ax17.scatter(vel_mag[::10], accel_mag[::10], force_mag[::10], alpha=0.6)
        ax17.set_xlabel('Velocity')
        ax17.set_ylabel('Acceleration')
        ax17.set_zlabel('Force')
        ax17.set_xlim([0.0,0.4])
        ax17.set_ylim([0.0,0.4])
        ax17.set_zlim([0.0,25.0])
        ax17.set_title('Magnitude')

        fig_name = 'Condition ' + str(mode) + ' Velocity vs Force'
        fig18 = plt.figure(fig_name)
        ax18 = fig18.add_subplot(111, projection='3d')
        scatter = ax18.scatter(aligned_vel[::10,0], aligned_vel[::10,1], aligned_vel[::10,2], c=force_mag[::10], alpha=0.6, cmap='viridis')
        color_bar = plt.colorbar(scatter, ax=ax18, shrink=0.5, aspect=5)
        color_bar.set_label('Force')
        ax18.set_xlabel('X Velocity')
        ax18.set_ylabel('Y Velocity')
        ax18.set_zlabel('Z Velocity')
        # ax18.set_xlim([0.0,0.4])
        # ax18.set_ylim([0.0,0.4])
        # ax18.set_zlim([0.0,25.0])
        ax18.set_title('Velocity Direction vs Force Magnitude')

# Prepare lists for coefficients
m_values_x = []
c_values_x = []
m_values_y = []
c_values_y = []
m_values_z = []
c_values_z = []
labels_x = []  # This will be used for labeling the bars
labels_y = []  # This will be used for labeling the bars
labels_z = []  # This will be used for labeling the bars

for ii in range(len(conditions)):
    # Extract the scalar values from the arrays
    if conditions[ii][-1] == 'x':
        m_values_x.append(mc_x_total[ii][0][0])  # Extract the scalar m from x
        c_values_x.append(mc_x_total[ii][1][0])  # Extract the scalar c from x
        labels_x.append(f'{conditions[ii]} (X)')

    if conditions[ii][-1] == 'y':
        m_values_y.append(mc_y_total[ii][0][0])  # Extract the scalar m from y
        c_values_y.append(mc_y_total[ii][1][0])  # Extract the scalar c from y
        labels_y.append(f'{conditions[ii]} (Y)')

    if conditions[ii][-1] == 'z':
        m_values_z.append(mc_z_total[ii][0][0])  # Extract the scalar m from z
        c_values_z.append(mc_z_total[ii][1][0])  # Extract the scalar c from z
        labels_z.append(f'{conditions[ii]} (Z)')

# m_values = []
# c_values = []
# labels = []  # This will be used for labeling the bars

# m_values.append(m_values_x)
# m_values.append(m_values_y)
# m_values.append(m_values_z)
# c_values.append(c_values_x)
# c_values.append(c_values_y)
# c_values.append(c_values_z)
# labels.append(labels_x)
# labels.append(labels_y)
# labels.append(labels_z)

# # Function to add value labels on the bars
# def autolabel(rects):
#     """Attach a text label above each bar in *rects*, displaying its height."""
#     for rect in rects:
#         height = rect.get_height()
#         ax.annotate(f'{height:.2f}',
#                     xy=(rect.get_x() + rect.get_width() / 2, height),
#                     xytext=(0, 3),  # 3 points vertical offset
#                     textcoords="offset points",
#                     ha='center', va='bottom')

# for i in range(len(m_values)):
#     # Create bar chart
#     x = np.arange(len(labels[i]))  # the label locations
#     width = 0.35  # the width of the bars

#     fig, ax = plt.subplots(figsize=(12, 6))
#     rects1 = ax.bar(x - width/2, m_values[i], width, label='m (Slope)')
#     rects2 = ax.bar(x + width/2, c_values[i], width, label='c (Intercept)')

#     # Add some text for labels, title and custom x-axis tick labels, etc.
#     ax.set_ylabel('Coefficient Values')
#     ax.set_title(f'Coefficient m and c at  {labels[i]}')
#     ax.set_xticks(x)
#     ax.set_xticklabels(labels[i], rotation=45, ha='right')
#     ax.legend()

#     autolabel(rects1)
#     autolabel(rects2)

# plt.tight_layout()
plt.show()






