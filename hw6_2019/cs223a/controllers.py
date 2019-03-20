#!/usr/bin/env python
"""
main.py

Authors: Toki Migimatsu
         Mingyu Wang
         Elena Galbally-Herrera
Created: March 2018
"""

import numpy as np
from dynamics import *
from jacobian import *
from links import *


###########################
# HW6 Q1 Inverse Dynamics #
###########################

# TODO: Implement 1 function below

def inverse_dynamics(links, q, dq, gains, q_des, dq_des=None, ddq_des=None):
    """
    PD inverse dynamics (joint space dynamics) control.

        tau = M(q) ddq + G(q)

    Args:
        links [Link, ...]: List of Link objects (see links.py)
        q             [N]: Numpy array of joint values (radians for revolute joints)
        dq            [N]: Numpy array of joint velocities (rad/s for revolute joints)
        gains      (Dict): PD gains { "kp_inv_dyn": Kp, "kv_inv_dyn": Kv }
        q_des         [N]: Numpy array of desired joint values
        dq_des        [N]: Numpy array of desired joint velocities (default 0)
        ddq_des       [N]: Numpy array of desired joint accelerations (default 0)

    Returns:
        tau [N]: Numpy array of control torques
    """
    dof = len(links)
    if dq_des is None:
        dq_des = np.zeros(dof)
    if ddq_des is None:
        ddq_des = np.zeros(dof)

    # Required gains
    Kp = gains["kp_inv_dyn"]
    Kv = gains["kv_inv_dyn"]

    # TODO: Replace follwing line with implementation

    M = mass_matrix(links, q)
    M = M.reshape((dof,dof))

    g = np.array([0, 0, -9.81])
    G = gravity_vector(links, q, g)
    G = G.reshape((dof,1))

    #print("SHAPE",q_des.shape)

    tau_prime = ddq_des.reshape((3,1)) - Kp*(q.reshape((3,1))-q_des.reshape((3,1))) - Kv*(dq.reshape((3,1)) - dq_des.reshape((3,1)))
    tau_prime = tau_prime.reshape((dof,1))

    tau_control = ((np.matmul(M,tau_prime)).reshape((dof,1)) + G).reshape(dof,1)
    tau_control = tau_control.reshape((dof,1))

    return tau_control

    #raise NotImplementedError("inverse_dynamics not implemented")


############################
# HW6 Q2 Operational Space #
############################

# TODO: Implement 1 function below

def nonredundant_operational_space(links, q, dq, gains, x_des, dx_des=np.zeros(3), ddx_des=np.zeros(3), ee_offset=np.zeros(3)):
    """
    Operational Space Control (xyz position) for non-redundant manipulators.

        F = M_x(q) ddx + G_x(q)
        tau = J_v^T F

    Args:
        links [Link, ...]: List of Link objects (see links.py)
        q             [N]: Numpy array of joint values (radians for revolute joints)
        dq            [N]: Numpy array of joint velocities (rad/s for revolute joints)
        gains      (Dict): PD gains { "kp_op_space": Kp, "kv_op_space": Kv }
        x_des         [3]: Numpy array of desired end-effector position
        xq_des        [3]: Numpy array of desired end-effector velocity (default 0)
        xdq_des       [3]: Numpy array of desired end-effector acceleration (default 0)
        ee_offset     [3]: Position of end-effector in the last link frame (default 0)

    Returns:
        tau [N]: Numpy array of control torques
    """
    dof = len(links)
    if dof > 3:
        raise ValueError("nonredundant_operational_space(): len(links) cannot be greater than 3.")

    # Required gains
    Kp = gains["kp_op_space"]
    Kv = gains["kv_op_space"]

    # TODO: Replace follwing line with implementation

    delta = 0   #for mass inaccuracy
    
    M = mass_matrix(links, q)
    M = M + delta*np.eye(dof)
    M = M.reshape((dof,dof))

    g = np.array([0, 0, -9.81])
    G = gravity_vector(links, q, g)
    G = G.reshape((dof,1))
    
    J = linear_jacobian(links, q, pos_in_link=ee_offset)

    rcond=1e-6  #for pseudoinverse computation

    #print("jacobian determinant", np.linalg.det(J))
    #U,S,V = np.linalg.svd(J)
    #print("sigma",S[0])

    Mx = np.matmul(np.matmul(np.linalg.pinv(J.T,rcond),M),np.linalg.pinv(J,rcond))
    Gx = np.matmul(np.linalg.pinv(J.T,rcond),G)

    #q = np.array([0,np.pi/2,25]) #for testing

    T = T_i_to_j(links,q,dof,0)


    x = (T[0:3,3]).reshape((3,1)) + (np.matmul(T[0:3,0:3],ee_offset)).reshape((3,1))
    x = x.reshape((3,1))
    
    #print("q",q)
    #print("x",x)

    #print("X",q, x.reshape((3,)))

    dx = np.matmul(J,dq)
    dx = dx.reshape((3,1))

    ddx = ddx_des.reshape((3,1)) - Kp*(x.reshape((3,1))-x_des.reshape((3,1))) - Kv*(dx.reshape((3,1)) - dx_des.reshape((3,1)))
    ddx = ddx.reshape((3,1))

    qdd = np.matmul(np.linalg.pinv(J,rcond),ddx)
    qdd = qdd.reshape((dof,1))

    tau = (np.matmul(M,qdd) + G).reshape((dof,1))

    #print("tau mag", np.linalg.norm(tau))

    



    return tau

    #raise NotImplementedError("nonredundant_operational_space not implemented")


#############################
# HW6 Q3 Inverse Kinematics #
#############################

def solve_qp(H, f=None, A=None, b=None, A_eq=None, b_eq=None):
    """
    Solve the QP:

        min 1/2 x^T H x + f^T x
        s.t.
            A    x  <= b
            A_eq x   = b_eq

    Args:
        H        [n x n]: Symmetric positive definite matrix
        f            [n]: Numpy array of size n
        A        [m x n]: Inequality constraint matrix
        b            [m]: Inequality constraint bias
        A_eq  [m_eq x n]: Equality constraint matrix
        b_eq      [m_eq]: Equality constraint bias
    """
    import quadprog

    n = H.shape[0]
    if f is None:
        f = np.zeros(n)
    if A is None and A_eq is None:
        return np.linalg.solve(H, -f)
    if A is None:
        A = np.zeros((0, n))
        b = np.zeros(0)
    if A_eq is None:
        A_eq = np.zeros((0, n))
        b_eq = np.zeros(0)
    m_eq = b_eq.shape[0]

    C = -np.hstack((A_eq.T, A.T))
    d = -np.hstack((b_eq, b))
    return quadprog.solve_qp(H, -f, C, d, m_eq)[0]


# TODO: Implement 1 function below

def inverse_kinematics(links, q, dq, gains, x_des, ee_offset=np.zeros(3), q_lim=(None, None)):
    """
    Velocity-based inverse kinematics control.

        Desired position will be converted into a desired velocity:

            dx_des = Kp * (x_des - x) / dt

        With joint limits, we solve the quadratic program:

            min || J_v dq_des - dx_des ||^2 + alpha || dq ||^2

            s.t. dq_des >= K_lim * (q_lim_lower - q) / dt
                 dq_des <= K_lim * (q_lim_upper - q) / dt

        Without joint limits, the problem gets reduced to:

            dq_des = J_v^{+} dx_des

        For a torque-controlled robot, the velocity will be converted into a
        torque using inverse dynamics:

            q_des  = q + dq_des * dt
            tau = inverse_dynamics(q_des, dq_des)

    Args:
        links        [Link, ...]: List of Link objects (see links.py)
        q                    [N]: Numpy array of joint values (radians for revolute joints)
        dq                   [N]: Numpy array of joint velocities (rad/s for revolute joints)
        gains             (Dict): IK gains { "kp_ik": Kp, k_joint_lim": K_lim, "ik_regularizer": alpha, "dt": dt }
        x_des                [3]: Numpy array of desired end-effector position
        ee_offset            [3]: Position of end-effector in the last link frame (default 0)
        q_lim (q_lower, q_upper): Tuple of lower and upper joint limits (default (None, None))

    Returns:
        tau [N]: Numpy array of control torques
    """
    dof = len(links)

    # Required gains
    Kp    = gains["kp_inv_kin"]
    K_lim = gains["k_joint_lim"]
    alpha = gains["ik_regularizer"]
    dt    = gains["dt"]


    # TODO: Replace follwing line with implementation

    J = linear_jacobian(links, q, pos_in_link=ee_offset)

    T = T_i_to_j(links,q,dof,0)
    x = T[0:3,3] + np.matmul(T[0:3,0:3],ee_offset)


    dx_des = (Kp * (x_des - x) / dt).reshape((3,))

    #print("speed limits",qdot_des_lower,qdot_des_upper)

    H = (np.matmul(J.T,J) + alpha*np.eye(3)).reshape((3,3))
    f = (-np.matmul(J.T,dx_des)).reshape((3,))

    #print("q_lim",q_lim[0],q_lim[1])

    no_upper_constraint = False
    no_lower_constraint = False


    if (not q_lim[0] is None) :
        qdot_des_lower = (K_lim*(q_lim[0] - q)/dt).reshape((dof,1))
    else:
        no_lower_constraint = True


    if (not q_lim[1] is None):
        qdot_des_upper = (K_lim*(q_lim[1] - q)/dt).reshape((dof,1))
    else:
        no_upper_constraint = True

    
    if(no_upper_constraint and no_lower_constraint):
        A = None
        b = None
        dq_des = solve_qp(H, f)
    elif (no_lower_constraint and not no_upper_constraint):
        A = np.eye(3)
        b = qdot_des_upper
        b = b.reshape((3,))
        dq_des = solve_qp(H, f, A, b)
    elif(not no_lower_constraint and no_upper_constraint):
        A = -np.eye(3)
        b = -qdot_des_lower
        b = b.reshape((3,))
        dq_des = solve_qp(H, f, A, b)
    else:
        A = np.vstack((np.eye(3),-np.eye(3)))
        b = np.vstack((qdot_des_upper,-qdot_des_lower))
        b = b.reshape(6,)
        dq_des = dq_des = solve_qp(H, f, A, b)



   

    #print("Ax",np.matmul(A,np.array([0,0,0])))
    #print("b",b)

    #print("matrixe",np.matmul(A,np.array([0,0,0])) - b)

    

    q_des  = q.reshape((3,)) + dq_des.reshape((3,)) * dt


    tau = inverse_dynamics(links, q, dq, gains, q_des, dq_des)

    return tau


    #raise NotImplementedError("inverse_kinematics not implemented")


