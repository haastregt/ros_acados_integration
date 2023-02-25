"""
Run this script a-priori to generate the code for the MPC solver. Some parameters
can be tuned to your desire.
"""

import numpy as np
import polytope as pc
import scipy
from astrobee import Astrobee
from acados_template import AcadosOcp, AcadosOcpSolver
from set_operations import SetOperations

"""
TUNEABLE PARAMETERS FOR THE SOLVER
"""
# Solver used
solver = "FULL_CONDENSING_HPIPM" # FULL_CONDENSING_QPOASES PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, 
                                # FULL_CONDENSING_HPIPM, PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, 
                                # FULL_CONDENSING_DAQP

MPC_HORIZON = 10 

terminalSetFilename = 'Xf.npy' # Name where terminal set is stored.
SET_TYPE = "LOAD" # LOAD, LQR or ZERO. LOAD loads from the filename, LQR or ZERO computes it and stores it in the filename

# Get the system dynamics (can be initialised with mass, inertia, and sampling time)
model = Astrobee(h=0.2) 

# State and Control cost matrices
Q = np.eye(12)
R = np.eye(6)

# Control and state bounds
u_lim = np.array([[0.85, 0.41, 0.41, 0.085, 0.041, 0.041]]).T
x_lim = np.array([[1.2, 0.1, 0.1,
                   0.5, 0.5, 0.5,
                   0.2, 0.2, 0.2,
                   0.1, 0.1, 0.1]]).T


"""
SETTING UP OF THE SOLVER AND GENERATING CODE
"""

def compute_terminal_set():
    A = model.Ad
    B = model.Bd

    # Translation Dynamics
    At = A[0:6, 0:6].reshape((6, 6))
    Bt = B[0:6, 0:3].reshape((6, 3))
    Qt = Q[0:6, 0:6].reshape((6, 6))
    Rt = R[0:3, 0:3].reshape((3, 3))
    x_lim_t = x_lim[0:6, :].reshape((6, 1))
    u_lim_t = u_lim[0:3, :].reshape((3, 1))
    set_ops_t = SetOperations(At, Bt, Qt, Rt, xlb=-x_lim_t, xub=x_lim_t)

    # Attitude Dynamics
    Aa = A[6:, 6:].reshape((6, 6))
    Ba = B[6:, 3:].reshape((6, 3))
    Qa = Q[6:, 6:].reshape((6, 6))
    Ra = R[3:, 3:].reshape((3, 3))
    x_lim_a = x_lim[6:, :].reshape((6, 1))
    u_lim_a = u_lim[3:, :].reshape((3, 1))
    set_ops_a = SetOperations(Aa, Ba, Qa, Ra, xlb=-x_lim_a, xub=x_lim_a)

    if SET_TYPE == "ZERO":
        Xf_t = set_ops_t.zeroSet()
        Xf_a = set_ops_a.zeroSet()
    elif SET_TYPE == "LQR":
        # Create constraint polytope for translation and attitude
        Cub = np.eye(3)
        Clb = -1 * np.eye(3)

        Cb_t = np.concatenate((u_lim_t, u_lim_t), axis=0)
        C_t = np.concatenate((Cub, Clb), axis=0)

        Cb_a = np.concatenate((u_lim_a, u_lim_a), axis=0)
        C_a = np.concatenate((Cub, Clb), axis=0)

        Ct = pc.Polytope(C_t, Cb_t)
        Ca = pc.Polytope(C_a, Cb_a)

        # Get the LQR set for each of these
        Xf_t = set_ops_t.LQRSet(Ct)
        Xf_a = set_ops_a.LQRSet(Ca)
    else:
        print("Wrong choice of SET_TYPE, select 'zero' or 'LQR'.")

    Xf = pc.Polytope(scipy.linalg.block_diag(Xf_t.A, Xf_a.A), np.concatenate((Xf_t.b, Xf_a.b), axis=0))
    
    return Xf

def save_xf_to_file(filename, Xf):
    with open(filename, 'wb') as f:
        np.save(f, Xf.A)
        np.save(f, Xf.b)

def load_xf_from_file(filename):
    with open(filename,'rb') as f:
        A = np.load(f)
        b = np.load(f)

    Xf = pc.Polytope(A,b)

    return Xf

def generate_solver_code():
    ocp = AcadosOcp()

    ocp.model = model.ExportAcadosModel()

    # Set Cost. EXTERAL allows us to define a casadi function for the cost
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL' #e is for end (timestep N)
    ocp.model.cost_expr_ext_cost = (ocp.model.x.T - ocp.model.p.T) @ Q @ (ocp.model.x - ocp.model.p) + ocp.model.u.T @ R @ ocp.model.u
    ocp.model.cost_expr_ext_cost_e = (ocp.model.x.T - ocp.model.p.T) @ P_LQR @ (ocp.model.x - ocp.model.p)
    
    # Nonlinear constraints in the form hl < h(x,u,p) < hu where h() is con_h_expr
    ocp.model.con_h_expr = ocp.model.x - ocp.model.p
    ocp.constraints.uh = np.squeeze(x_lim)
    ocp.constraints.lh = np.squeeze(-x_lim)

    # Control constraints in the form lbu < Ju < ubu
    ocp.constraints.Jbu = np.eye(model.m)
    ocp.constraints.ubu = np.squeeze(u_lim)
    ocp.constraints.lbu = np.squeeze(-u_lim)

    # Terminal constraint
    ocp.model.con_h_expr_e = Xf.A @ (ocp.model.x - ocp.model.p)
    ocp.constraints.uh_e = np.squeeze(Xf.b)
    ocp.constraints.lh_e = -1E15*np.ones(np.size(np.squeeze(Xf.b))) # No Inf support

    # This is to initialise the dimensionality. Will be set to actual state when calling solve
    ocp.constraints.x0 = np.zeros((model.n,))
    ocp.parameter_values = np.zeros((model.n,))

    # set options
    ocp.solver_options.qp_solver = solver 
    ocp.solver_options.hessian_approx = 'EXACT' # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = 'ERK' #'IRK', 'ERK', 'DISCRETE'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI' # SQP_RTI, SQP

    ocp.dims.N = MPC_HORIZON # Prediction horizon
    ocp.solver_options.tf = MPC_HORIZON*model.dt # Final time

    AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')

if __name__ == "__main__":
    # Solve the ARE for our system to extract the terminal weight matrix P
    P_LQR = np.matrix(scipy.linalg.solve_discrete_are(model.Ad, model.Bd, Q, R))

    # Load terminal set
    if SET_TYPE == "LOAD":
        Xf = load_xf_from_file(terminalSetFilename)
    else:
        Xf = compute_terminal_set()
        save_xf_to_file(terminalSetFilename, Xf)

    generate_solver_code()