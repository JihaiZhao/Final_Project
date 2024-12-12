import numpy as np
import csv
import matplotlib.pyplot as plt
from sklearn.mixture import GaussianMixture
rng = np.random.default_rng(1)

np.set_printoptions(precision=4)

from tqdm import tqdm
import sys
from scipy.optimize import minimize

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

class iLQR_template:
    def __init__(self, dt, tsteps, x_dim, u_dim, Q_z, R_v) -> None:
        self.dt = dt
        self.tsteps = tsteps

        self.x_dim = x_dim
        self.u_dim = u_dim

        self.Q_z = Q_z
        self.Q_z_inv = np.linalg.inv(Q_z)
        self.R_v = R_v
        self.R_v_inv = np.linalg.inv(R_v)

        self.curr_x_traj = None
        self.curr_y_traj = None

    def dyn(self, xt, ut):
        raise NotImplementedError("Not implemented.")

    def step(self, xt, ut):
        """RK4 integration"""
        k1 = self.dt * self.dyn(xt, ut)
        k2 = self.dt * self.dyn(xt + k1/2.0, ut)
        k3 = self.dt * self.dyn(xt + k2/2.0, ut)
        k4 = self.dt * self.dyn(xt + k3, ut)

        xt_new = xt + (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0
        return xt_new

    def traj_sim(self, x0, u_traj):
        x_traj = np.zeros((self.tsteps, self.x_dim))
        xt = x0.copy()
        for t_idx in range(self.tsteps):
            xt = self.step(xt, u_traj[t_idx])
            # xt =  minimize(self.cost, u0, args=(x0, dt, 10, ref), bounds=u_bounds, method='SLSQP')
            x_traj[t_idx] = xt.copy()
        #     print(xt, flush = True)

        # sys.exit()
        return x_traj

    def loss(self):
        raise NotImplementedError("Not implemented.")

    def get_At_mat(self, t_idx):
        raise NotImplementedError("Not implemented.")

    def get_Bt_mat(self, t_idx):
        raise NotImplementedError("Not implemented.")

    def get_at_vec(self, t_idx):
        raise NotImplementedError("Not implemented.")

    def get_bt_vec(self, t_idx):
        raise NotImplementedError("Not implemented.")

    # the following functions are utilities for solving the Riccati equation
    def P_dyn_rev(self, Pt, At, Bt, at, bt):
        return Pt @ At + At.T @ Pt - Pt @ Bt @ self.R_v_inv @ Bt.T @ Pt + self.Q_z

    def P_dyn_step(self, Pt, At, Bt, at, bt):
        k1 = self.dt * self.P_dyn_rev(Pt, At, Bt, at, bt)
        k2 = self.dt * self.P_dyn_rev(Pt+k1/2, At, Bt, at, bt)
        k3 = self.dt * self.P_dyn_rev(Pt+k2/2, At, Bt, at, bt)
        k4 = self.dt * self.P_dyn_rev(Pt+k3, At, Bt, at, bt)

        Pt_new = Pt + (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0
        return Pt_new

    def P_traj_revsim(self, PT, A_traj, B_traj, a_traj, b_traj):
        P_traj_rev = np.zeros((self.tsteps, self.x_dim, self.x_dim))
        P_curr = PT.copy()
        for t in range(self.tsteps):
            At = A_traj[-1-t]
            Bt = B_traj[-1-t]
            at = a_traj[-1-t]
            bt = b_traj[-1-t]

            P_new = self.P_dyn_step(P_curr, At, Bt, at, bt)
            P_traj_rev[t] = P_new.copy()
            P_curr = P_new

        return P_traj_rev

    def r_dyn_rev(self, rt, Pt, At, Bt, at, bt):
        return (At - Bt @ self.R_v_inv @ Bt.T @ Pt).T @ rt + at - Pt @ Bt @ self.R_v_inv @ bt

    def r_dyn_step(self, rt, Pt, At, Bt, at, bt):
        k1 = self.dt * self.r_dyn_rev(rt, Pt, At, Bt, at, bt)
        k2 = self.dt * self.r_dyn_rev(rt+k1/2, Pt, At, Bt, at, bt)
        k3 = self.dt * self.r_dyn_rev(rt+k2/2, Pt, At, Bt, at, bt)
        k4 = self.dt * self.r_dyn_rev(rt+k3, Pt, At, Bt, at, bt)

        rt_new = rt + (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0
        return rt_new

    def r_traj_revsim(self, rT, P_traj, A_traj, B_traj, a_traj, b_traj):
        r_traj_rev = np.zeros((self.tsteps, self.x_dim))
        r_curr = rT
        for t in range(self.tsteps):
            Pt = P_traj[-1-t]
            At = A_traj[-1-t]
            Bt = B_traj[-1-t]
            at = a_traj[-1-t]
            bt = b_traj[-1-t]

            r_new = self.r_dyn_step(r_curr, Pt, At, Bt, at, bt)
            r_traj_rev[t] = r_new.copy()
            r_curr = r_new

        return r_traj_rev

    def z_dyn(self, zt, Pt, rt, At, Bt, bt):
        return At @ zt + Bt @ self.z2v(zt, Pt, rt, Bt, bt)

    def z_dyn_step(self, zt, Pt, rt, At, Bt, bt):
        k1 = self.dt * self.z_dyn(zt, Pt, rt, At, Bt, bt)
        k2 = self.dt * self.z_dyn(zt+k1/2, Pt, rt, At, Bt, bt)
        k3 = self.dt * self.z_dyn(zt+k2/2, Pt, rt, At, Bt, bt)
        k4 = self.dt * self.z_dyn(zt+k3, Pt, rt, At, Bt, bt)

        zt_new = zt + (k1 + 2.0*k2 + 2.0*k3 + k4) / 6.0
        return zt_new

    def z_traj_sim(self, z0, P_traj, r_traj, A_traj, B_traj, b_traj):
        z_traj = np.zeros((self.tsteps, self.x_dim))
        z_curr = z0.copy()

        for t in range(self.tsteps):
            Pt = P_traj[t]
            rt = r_traj[t]
            At = A_traj[t]
            Bt = B_traj[t]
            bt = b_traj[t]

            z_new = self.z_dyn_step(z_curr, Pt, rt, At, Bt, bt)
            z_traj[t] = z_new.copy()
            z_curr = z_new

        return z_traj

    def z2v(self, zt, Pt, rt, Bt, bt):
        return -self.R_v_inv @ Bt.T @ Pt @ zt - self.R_v_inv @ Bt.T @ rt - self.R_v_inv @ bt

    def get_descent(self, x0, u_traj):
        # forward simulate the trajectory
        x_traj = self.traj_sim(x0, u_traj)
        self.curr_x_traj = x_traj.copy()
        self.curr_u_traj = u_traj.copy()

        # sovle the Riccati equation backward in time
        A_traj = np.zeros((self.tsteps, self.x_dim, self.x_dim))
        B_traj = np.zeros((self.tsteps, self.x_dim, self.u_dim))
        a_traj = np.zeros((self.tsteps, self.x_dim))
        b_traj = np.zeros((self.tsteps, self.u_dim))

        for t_idx in range(self.tsteps):
            A_traj[t_idx] = self.get_At_mat(t_idx)
            B_traj[t_idx] = self.get_Bt_mat(t_idx)
            a_traj[t_idx] = self.get_at_vec(t_idx)
            b_traj[t_idx] = self.get_bt_vec(t_idx)

        # print('a_traj:\n', a_traj)

        PT = np.zeros((self.x_dim, self.x_dim))
        P_traj_rev = self.P_traj_revsim(PT, A_traj, B_traj, a_traj, b_traj)
        P_traj = np.flip(P_traj_rev, axis=0)

        rT = np.zeros(self.x_dim)
        r_traj_rev = self.r_traj_revsim(rT, P_traj, A_traj, B_traj, a_traj, b_traj)
        r_traj = np.flip(r_traj_rev, axis=0)

        z0 = np.zeros(self.x_dim)
        z_traj = self.z_traj_sim(z0, P_traj, r_traj, A_traj, B_traj, b_traj)

        # compute the descent direction
        v_traj = np.zeros((self.tsteps, self.u_dim))
        for t in range(self.tsteps):
            zt = z_traj[t]
            Pt = P_traj[t]
            rt = r_traj[t]
            Bt = B_traj[t]
            bt = b_traj[t]
            v_traj[t] = self.z2v(zt, Pt, rt, Bt, bt)

        return v_traj

input_files = ['s2_after.csv', 's3_after.csv', 's4_after.csv', 's5_after.csv', 's6_after.csv', 's7_after.csv', 's8_after.csv']
# input_false_files = ['w_s1_after.csv', 'w_s2_after.csv', 'w_s3_after.csv', 'w_s4_after.csv']

def collect_data(filename, step=150):
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        header = next(reader)
        rows = list(reader)

        if len(rows) > 0:
            first_row = [float(x) for x in rows[0]]
            last_row = [float(x) for x in rows[-1]]

            collect_data = False
            collected_rows = []
            row_counter = 0

            for row in rows:
                current_row = [float(x) for x in row]

                if abs(current_row[0] - first_row[0]) > 0.005 and abs(current_row[1] - first_row[1]) > 0.005 and not collect_data:
                    collect_data = True

                if collect_data:
                    if row_counter % step == 0:
                        collected_rows.append(row)
                    row_counter += 1

                if abs(current_row[0] - last_row[0]) < 0.005 and abs(current_row[1] - last_row[1]) < 0.005:
                    break

            return collected_rows
        else:
            return []

all_data = []

for input_file in input_files:
    filtered_data = collect_data(input_file, step=150)
    x_values = [float(row[0]) for row in filtered_data]
    y_values = [float(row[1])+0.5 for row in filtered_data]
    all_data.extend(list(zip(y_values, x_values)))

X = np.array(all_data)

# all_data_false = []

# for input_file in input_false_files:
#     filtered_data = collect_data(input_file, step=100)
#     x_values = [float(row[0]) for row in filtered_data]
#     y_values = [float(row[1])+0.5 for row in filtered_data]
#     all_data_false.extend(list(zip(y_values, x_values)))

# X_false = np.array(all_data_false)

# Apply Gaussian Mixture Model
n_components = 7  # You can adjust this number
gmm = GaussianMixture(n_components=n_components, random_state=42)
gmm_labels = gmm.fit_predict(X)

# # Apply Gaussian Mixture Model to false data sets
# n_components = 5  # You can adjust this number
# gmm_false = GaussianMixture(n_components=n_components, random_state=42)
# gmm_labels_false = gmm_false.fit_predict(X_false)

# Plotting
plt.figure(figsize=(10, 8))

# Plot the clustered data
scatter = plt.scatter(X[:, 0], X[:, 1], c=gmm_labels, cmap='viridis', alpha=0.7)

# Plot the cluster centers
centers = gmm.means_
plt.scatter(centers[:, 0], centers[:, 1], c='red', s=200, alpha=0.8, marker='x', label='Cluster Centers')

# # Plot the cluster centers
# centers_false = gmm_false.means_
# plt.scatter(centers_false[:, 0], centers_false[:, 1], c='red', s=200, alpha=0.8, marker='x', label='Cluster Centers')

# Set the axis limits
plt.xlim(0, 1)
plt.ylim(0, 1)

# Set the grid with specific spacing
plt.xticks([i * 0.05 for i in range(21)], fontsize = 8)
plt.yticks([i * 0.05 for i in range(21)], fontsize = 8)
plt.grid(True)

# Set aspect ratio to equal to maintain the proportions
plt.gca().set_aspect('auto')

# Add plot details
plt.xlabel('Y')
plt.ylabel('X')
plt.title('Gaussian Mixture Model Clustering of Collected Data')

# Add a legend
plt.legend()

# Show the plot
plt.show()

# Count the occurrences of each label
unique_labels, label_counts = np.unique(gmm_labels, return_counts=True)

# Print the counts of each label
for label, count in zip(unique_labels, label_counts):
    print(f"Label {label}: {count} points")

def make_psd(cov_matrix, epsilon=1e-6):
    # Ensure the covariance matrix is symmetric
    cov_matrix = (cov_matrix + cov_matrix.T) / 2
    # Perform eigenvalue decomposition
    eigvals, eigvecs = np.linalg.eigh(cov_matrix)
    # Add epsilon to eigenvalues to make all of them positive
    eigvals = np.maximum(eigvals, epsilon)
    # Reconstruct the covariance matrix
    pd_matrix = eigvecs @ np.diag(eigvals) @ eigvecs.T
    return pd_matrix

### We define a Gaussian mixture model as the spatial probability density function
from scipy.stats import multivariate_normal as mvn

mean1 = np.array(centers[0])
cov1 = np.array([
    [0.005, -0.003],
    [-0.003, 0.005]
])
w1 = 0.1

mean2 = np.array(centers[1])
cov2 = np.array([
    [0.01, 0.004],
    [0.004, 0.01]
])
w2 = 0.2

# Make cov2 positive semidefinite
cov2_psd = make_psd(cov2)
print(cov2_psd)

mean3 = np.array(centers[2])
cov3 = np.array([
    [0.008, 0.0],
    [0.0, 0.004]
])
w3 = 0.1

# Make cov2 positive semidefinite
cov3_psd = make_psd(cov3)
print(cov3_psd)

mean4 = np.array(centers[3])
cov4 = np.array([
    [0.008, 0.0],
    [0.0, 0.004]
])
w4 = 0.2

# Make cov2 positive semidefinite
cov4_psd = make_psd(cov4)
print(cov4_psd)

mean5 = np.array(centers[4])
cov5 = np.array([
    [0.005, -0.003],
    [-0.003, 0.005]
])
w5 = 0.2

# Make cov2 positive semidefinite
cov5_psd = make_psd(cov5)
print(cov5_psd)

mean6 = np.array(centers[5])
cov6 = np.array([
    [0.01, 0.004],
    [0.004, 0.01]
])
w6 = 0.1

# Make cov6 positive semidefinite
cov6_psd = make_psd(cov6)
print(cov6_psd)

mean7 = np.array(centers[6])
cov7 = np.array([
    [0.005, -0.003],
    [-0.003, 0.005]
])
w7 = 0.1

def pdf(x):
    return w1 * mvn.pdf(x, mean1, cov1) + \
           w2 * mvn.pdf(x, mean2, cov2) + \
           w3 * mvn.pdf(x, mean3, cov3) + \
           w4 * mvn.pdf(x, mean4, cov4) + \
           w5 * mvn.pdf(x, mean5, cov5) + \
           w6 * mvn.pdf(x, mean6, cov6) + \
           w7 * mvn.pdf(x, mean7, cov7)

centers

### We are going to use 190 coefficients per dimension --- so 100 index vectors in total

num_k_per_dim = 10
ks_dim1, ks_dim2 = np.meshgrid(
    np.arange(num_k_per_dim), np.arange(num_k_per_dim)
)
ks = np.array([ks_dim1.ravel(), ks_dim2.ravel()]).T  # this is the set of all index vectors

# Define a 1-by-1 2D search space
L_list = np.array([1.0, 1.0])  # boundaries for each dimension

# Discretize the search space into 100-by-100 mesh grids
grids_x, grids_y = np.meshgrid(
    np.linspace(0, L_list[0], 100),
    np.linspace(0, L_list[1], 100)
)
grids = np.array([grids_x.ravel(), grids_y.ravel()]).T
dx = 1.0 / 99
dy = 1.0 / 99

# Pre-processing lambda_k and h_k
lamk_list = np.power(1.0 + np.linalg.norm(ks, axis=1), -3/2.0)
hk_list = np.zeros(ks.shape[0])

coefficients = np.zeros(ks.shape[0])
# Compute the coefficients
for i, k_vec in enumerate(ks):
    fk_vals = np.prod(np.cos(np.pi * k_vec / L_list * grids), axis=1)
    hk = np.sqrt(np.sum(np.square(fk_vals)) * dx * dy)
    a = fk_vals/hk
    pdf_a = pdf(grids)
    b = np.sum(a*pdf_a) * dx * dy
    coefficients[i] = b

    hk_list[i] = hk

# compute the coefficients for the target distribution
phik_list = np.zeros(ks.shape[0])
pdf_vals = pdf(grids)
for i, (k_vec, hk) in enumerate(zip(ks, hk_list)):
    fk_vals = np.prod(np.cos(np.pi * k_vec / L_list * grids), axis=1)
    fk_vals /= hk

    phik = np.sum(fk_vals * pdf_vals) * dx * dy
    phik_list[i] = phik

pdf_gt = pdf(grids)  # ground truth density function

# visualize for comparison
fig, axes = plt.subplots(1, 1, figsize=(9,5), dpi=70, tight_layout=True)

ax = axes
ax.set_aspect('equal')
ax.set_xlim(0, L_list[0])
ax.set_ylim(0, L_list[1])
ax.set_title('Original PDF')
contour = ax.contourf(grids_x, grids_y, pdf_gt.reshape(grids_x.shape), cmap='Reds')

plt.colorbar(contour, ax=ax)
plt.show()
plt.close()

coefficients.shape

phik_list

class iLQR_ergodic_robotarm(iLQR_template):
    def __init__(self, dt, tsteps, x_dim, u_dim, Q_z, R_v,
                 R, ks, L_list, lamk_list, hk_list, phik_list) -> None:
        super().__init__(dt, tsteps, x_dim, u_dim, Q_z, R_v)

        self.R = R
        self.ks = ks
        self.L_list = L_list
        self.lamk_list = lamk_list
        self.hk_list = hk_list
        self.phik_list = phik_list

    def dyn(self, xt, ut):
        xdot = np.array([
            xt[2],
            xt[3],
            ut[0],
            ut[1]
        ])
        return xdot

    def get_At_mat(self, t_idx):
        # xt = self.curr_x_traj[t_idx]
        # ut = self.curr_u_traj[t_idx]
        A = np.array([
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]
        ])
        return A

    def get_Bt_mat(self, t_idx):
        # xt = self.curr_x_traj[t_idx]
        B = np.array([
            [0.0, 0.0],
            [0.0, 0.0],
            [1.0, 0.0],
            [0.0, 1.0]
        ])
        return B

    def get_at_vec(self, t_idx):
        xt = self.curr_x_traj[t_idx][:2]
        x_traj = self.curr_x_traj[:,:2]

        dfk_xt_all = np.array([
            -np.pi * self.ks[:,0] / self.L_list[0] * np.sin(np.pi * self.ks[:,0] / self.L_list[0] * xt[0]) * np.cos(np.pi * self.ks[:,1] / self.L_list[1] * xt[1]),
            -np.pi * self.ks[:,1] / self.L_list[1] * np.cos(np.pi * self.ks[:,0] / self.L_list[0] * xt[0]) * np.sin(np.pi * self.ks[:,1] / self.L_list[1] * xt[1]),
        ]) / self.hk_list

        fk_all = np.prod(np.cos(np.pi * self.ks / self.L_list * x_traj[:,None]), axis=2) / self.hk_list
        ck_all = np.sum(fk_all, axis=0) * self.dt / (self.tsteps * self.dt)

        at = np.sum(self.lamk_list * 2.0 * (ck_all - self.phik_list) * dfk_xt_all / (self.tsteps * self.dt), axis=1)
        return np.array([at[0], at[1], 0.0, 0.0])

    def get_bt_vec(self, t_idx):
        ut = self.curr_u_traj[t_idx]
        return self.R @ ut

    def loss(self, x_traj, u_traj):
        fk_all = np.prod(np.cos(np.pi * self.ks / self.L_list * x_traj[:,:2][:,None]), axis=2) / self.hk_list
        ck_all = np.sum(fk_all, axis=0) * self.dt / (self.tsteps * self.dt)
        erg_metric = np.sum(self.lamk_list * np.square(ck_all - self.phik_list))

        ctrl_cost = np.sum(self.R @ u_traj.T * u_traj.T) * self.dt
        return erg_metric + ctrl_cost

# Define the optimal control problem
dt = 0.1
tsteps = 100
R = np.diag([0.01, 0.008])
Q_z=np.diag([0.01, 0.008, 0.008, 0.008])
# R_v=np.diag([0.07, 0.06])
R_v=np.diag([0.02, 0.015])
x0 = np.zeros(4)

# x0[:2] = rng.uniform(low=0.3, high=0.4, size=(2,))
x0[0] = -0.2 + 0.3
x0[1] = 0.4
init_u_traj = np.tile(np.array([0.005, 0.0015]), reps=(tsteps,1))

trajopt_ergodic_robotarm = iLQR_ergodic_robotarm(
    dt, tsteps, x_dim=4, u_dim=2, Q_z=Q_z, R_v=R_v,
    R=R, ks=ks, L_list=L_list, lamk_list=lamk_list,
    hk_list=hk_list, phik_list=phik_list
)

# Iterative trajectory optimization for ergodic control
from IPython import display

u_traj = init_u_traj.copy()
step = 0.01
loss_list = []

fig, axes = plt.subplots(1, 3, dpi=70, figsize=(25,5), tight_layout=True)

for iter in tqdm(range(100)):
    x_traj = trajopt_ergodic_robotarm.traj_sim(x0, u_traj)
    v_traj = trajopt_ergodic_robotarm.get_descent(x0, u_traj)

    loss_val = trajopt_ergodic_robotarm.loss(x_traj, u_traj)
    loss_list.append(loss_val)

    step = 0.001
    alpha = 0.5
    for _i in range(3):
        temp_u_traj = u_traj + step * v_traj
        temp_x_traj = trajopt_ergodic_robotarm.traj_sim(x0, temp_u_traj)
        temp_loss_val = trajopt_ergodic_robotarm.loss(temp_x_traj, temp_u_traj)
        if temp_loss_val < loss_val:
            break
        else:
            step *= alpha
    u_traj += step * v_traj

    # visualize every 10 iterations
    if (iter+1) % 10 == 0:
        ax1 = axes[0]
        ax1.cla()
        ax1.set_aspect('equal', adjustable='box')
        ax1.set_xlim(0.0, L_list[0])
        ax1.set_ylim(0.0, L_list[1])
        ax1.set_title('Iteration: {:d}'.format(iter+1))
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.contourf(grids_x, grids_y, pdf_vals.reshape(grids_x.shape), cmap='Reds')
        # ax1.plot([x0[0], x_traj[0,0]], [x0[1], x_traj[0,1]], linestyle='-', linewidth=2, color='k', alpha=1.0)
        ax1.plot(x_traj[:,0], x_traj[:,1], linestyle='-', marker='o', color='k', linewidth=2, alpha=1.0, label='Optimized trajectory')
        ax1.plot(x0[0], x0[1], linestyle='', marker='o', markersize=15, color='C0', alpha=1.0, label='Initial state')
        ax1.legend(loc=1)

        ax2 = axes[1]
        ax2.cla()
        ax2.set_title('Control vs. Time')
        ax2.set_ylim(-1.5, 2.5)
        ax2.plot(np.arange(tsteps)*dt, u_traj[:,0], color='C0', label=r'$u_1$')
        ax2.plot(np.arange(tsteps)*dt, u_traj[:,1], color='C1', label=r'$u_2$')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Control')
        ax2.legend(loc=1)
        height = ax1.get_position().height
        ax2.set_position([ax2.get_position().x0, ax1.get_position().y0, ax2.get_position().width, height])

        ax3 = axes[2]
        ax3.cla()
        ax3.set_title('Objective vs. Iteration')
        ax3.set_xlim(-0.2, 100.2)
        ax3.set_ylim(9e-2, 1.1)
        ax3.set_xlabel('Iteration')
        ax3.set_ylabel('Objective')
        ax3.plot(np.arange(iter+1), loss_list, color='C3')
        height = ax1.get_position().height
        ax3.set_position([ax3.get_position().x0, ax1.get_position().y0, ax3.get_position().width, height])
        ax3.set_yscale('log')

        # plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=0.1, hspace=0.1)
        display.clear_output(wait=True)
        display.display(fig)

display.clear_output(wait=True)
plt.show()
plt.close()

plt.plot(x_traj[:,0],x_traj[:,1])

from os import write
import csv
import matplotlib.pyplot as plt


# Function to save the extracted data to a CSV file
with open("trajectory_data.csv", mode='w', newline='') as csvfile:
    writer = csv.writer(csvfile)

    # write x0
    writer.writerow([x0[0]-0.3, x0[1]])

    # Write the values
    for entry in x_traj:
        writer.writerow([entry[0]-0.3, entry[1]])