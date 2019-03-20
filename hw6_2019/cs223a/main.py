#!/usr/bin/env python
"""
main.py

Authors: Toki Migimatsu
         Mingyu Wang
         Elena Galbally-Herrera
Created: December 2017
"""

import numpy as np
import time
from robot import Robot
from controllers import *
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt


# Frequency constants
FREQ_SIMULATION = 1000 # Simulation frequency [Hz]
FREQ_CONTROL    = 200  # Controller frequency [Hz] (should be lower than sim)


def q1_q2_robot():
    robot = Robot(freq_sim=FREQ_SIMULATION, freq_control=FREQ_CONTROL)

    # Add links
    robot.add_revolute(a       = 0,
                        alpha   = 0,
                        d       = 0.0,
                        mass    = 0,
                        com     = np.array([0,0,0]),
                        inertia = np.zeros((3,3)))
    robot.add_revolute(a       = 0,
                        alpha   = np.pi/2.0,
                        d       = 0.0,
                        mass    = 1,
                        com     = np.array([0,-0.15,0]),
                        inertia = np.eye(3))
    robot.add_prismatic(a       = 0,
                        alpha   = np.pi/2.0,
                        theta   = 0,
                        mass    = 1,
                        com     = np.array([0,0,0.15]),
                        inertia = np.eye(3))

    # Add end-effector
    robot.set_ee_offset(np.array([0, 0, 0.3]))  # end-effector offset

    # Set (lower, upper) joint limits
    robot.set_joint_limits(np.array([-np.pi, -np.pi, 0]), np.array([np.pi, np.pi, 2]))

    # Set initial configuration
    robot.simulator.set_configuration(np.zeros(3))

    return robot

def q3_robot():
    robot = Robot(freq_sim=FREQ_SIMULATION, freq_control=FREQ_CONTROL)

    # Add links
    robot.add_revolute (a       = 0,
                        alpha   = 0,
                        d       = 0.0,
                        mass    = 0,
                        com     = np.array([0,0,0]),
                        inertia = np.zeros((3,3)))
    robot.add_revolute (a       = 0.2,
                        alpha   = np.pi/2.0,
                        d       = 0.0,
                        mass    = 1,
                        com     = np.array([0,0,-0.1]),
                        inertia = np.eye(3))
    robot.add_prismatic(a       = 0,
                        alpha   = np.pi/2.0,
                        theta   = 0,
                        mass    = 1,
                        com     = np.array([0,0,0]),
                        inertia = np.zeros((3,3)))

    # Add end-effector
    robot.set_ee_offset(np.array([0, 0, 0.1]))

    # Set (lower, upper) joint limits
    robot.set_joint_limits(np.array([-np.pi, -np.pi, 0]), np.array([np.pi, np.pi, 2]))

    # Set initial configuration
    robot.simulator.set_configuration(np.array([0, np.pi/2, 0]))

    return robot


def q1_trajectory(t):
    return np.array([np.sin(t), t, 0.25*t])

def q2_trajectory(t):
    return np.array([(0.5*t+0.25)*np.sin(t), 0, -(0.5*t+0.25)*np.cos(t)])

def q3_trajectory(t):
    return np.array([0.5, 0.2*np.sin(np.pi/2*t), 0.05*t])

if __name__ == "__main__":
    # Time constants
    T_SIMULATION = 5  # Simulation duration [s]
    NUM_ITERS = int(T_SIMULATION * FREQ_CONTROL)  # Number of controller iterations

    #################
    ### Set gains ###
    #################

    # TODO: Tune gains
    gains = {
        # Inverse dynamics
        "kp_inv_dyn": 49,
        "kv_inv_dyn": 14,

        # Operational Space
        "kp_op_space": 49,
        "kv_op_space": 14,

        # Inverse kinematics
        "kp_inv_kin": 1,
        "k_joint_lim": 1,
        "ik_regularizer": 10.0,
        "dt": 1. / FREQ_CONTROL,
    }



    ####################
    ### Create robot ###
    ####################

    # TODO: Select robot

    #robot = q1_q2_robot()
    robot = q3_robot()

    #for problem 1b
    '''
    robot.set_gravity()
    q_config = np.array([ [0],[(180)*np.pi/180],[100]  ])
    g_vec = robot.gravity_vector(q_config)
    print("gravity vector ",g_vec)
    '''

    # Record data for plotting
    FREQ_PLOTTING = 10
    plotting_data = {
        "q":   [],
        "dq":  [],
        "x":   [],
        "dx":  [],
        "tau": []
    }

    # Run for T_SIMULATION seconds
    t_start = time.time()
    for i in range(NUM_ITERS):
        # Retrieve robot state from simulation
        q, dq = robot.read_sensor_values()
        t_sim = float(i) / FREQ_CONTROL

        ############################
        ### Set desired position ###
        ############################

        # TODO: Select trajectory

        # q_des  = np.array([0., np.pi/2.0, 0])
        # x_des = np.array([0.15, 0.25, 0.35])
        #q_des = q1_trajectory(t_sim)
        x_des = q2_trajectory(t_sim)
        #x_des = q3_trajectory(t_sim)



        ###############################
        ### Compute control torques ###
        ###############################

        # TODO: Select controller

        #tau = inverse_dynamics(robot.links, q, dq, gains, q_des)
        #tau = nonredundant_operational_space(robot.links, q, dq, gains, x_des, ee_offset=robot.ee_offset)
        tau = inverse_kinematics(robot.links, q, dq, gains, x_des, ee_offset=robot.ee_offset, q_lim=robot.joint_limits)



        # Send command torques and advance one simulation step
        robot.send_command_torques(tau)

        # Any values passed into this function will be displayed in the web interface
        x  = robot.ee_pos(q)
        dx = robot.ee_vel(q, dq)
        robot.publish_values({
            "q":   q,
            "dq":  dq,
            "x":   x,
            "dx":  dx,
            "tau": tau
        })
        # print("q",q)
        # Record data for plotting
        if i % (FREQ_CONTROL // FREQ_PLOTTING) == 0:
            plotting_data["q"].append(q)
            plotting_data["dq"].append(dq)
            plotting_data["x"].append(x)
            plotting_data["dx"].append(dx)
            plotting_data["tau"].append(tau)

        # Slow down simulation to real time
        t_next = t_start + (i + 1.) / FREQ_CONTROL
        t_real = time.time()
        if t_real < t_next:
            time.sleep(t_next - t_real)

    print("Simulated {0:f}s in {1}s.".format(T_SIMULATION, time.time() - t_start))



    #########################
    ### Plot trajectories ###
    #########################

    for key in plotting_data:
        plotting_data[key] = np.column_stack(plotting_data[key])

    # TODO: Generate plots. An example plot is given to you below.
    plt.figure()

    plt.title("Joint Position Trajectory")
    plt.ylabel("q(t)")
    plt.xlabel("time [s]")
    for i in range(robot.dof()):
        plt.plot(plotting_data["q"][i,:])
    plt.legend(["q_" + str(i+1) for i in range(robot.dof())])
    idx_sec = range(0, T_SIMULATION * FREQ_PLOTTING + 1, FREQ_PLOTTING)
    plt.xticks(idx_sec, [i for i in range(len(idx_sec))])


    plt.figure()


    plt.title("Inverse kinematics Controller, Alpha= " + str(gains["ik_regularizer"]))
    plt.ylabel("x(t)")
    plt.xlabel("time [s]")
    
    
    for i in range(3):
        plt.plot(plotting_data["x"][i,:])

    plt.legend([ "x1","x2","x3"])
    
    t = np.linspace(0.0,5.0,50)
    x_des = np.zeros((t.shape[0],3))
    #([0.5, , 0.05*t])
    #q2 np.array([(0.5*t+0.25)*np.sin(t), 0, -(0.5*t+0.25)*np.cos(t)])
    # q3 np.array([0.5, 0.2*np.sin(np.pi/2*t), 0.05*t])
    x_des[:,0] = (0.5*t+0.25)*np.sin(t)#0.5
    x_des[:,1] = 0#0.2*np.sin(np.pi/2*t)
    x_des[:,2] = -(0.5*t+0.25)*np.cos(t)#0.05*t

    for i in range(3):
        plt.plot(x_des[:,i],'--')
    

    plt.legend([ "x1","x2","x3","x1_des","x2_des","x3_des" ])



    plt.figure()


    plt.title("Inverse kinematics Controller, Alpha= " + str(gains["ik_regularizer"]))
    plt.ylabel("q(t)")
    plt.xlabel("time [s]")
    
    
    for i in range(3):
        plt.plot(plotting_data["q"][i,:])

    plt.legend([ "q1","q2","q3"])
    


    

    idx_sec = range(0, T_SIMULATION * FREQ_PLOTTING + 1, FREQ_PLOTTING)
    plt.xticks(idx_sec, [i for i in range(len(idx_sec))])

    plt.show()
