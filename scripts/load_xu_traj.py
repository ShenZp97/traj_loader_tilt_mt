#!/usr/bin/env python

import numpy as np
import os
import rospkg
import rospy
import yaml

from aerial_robot_msgs.msg import PredXU
import argparse
from std_msgs.msg import MultiArrayDimension


# read parameters from yaml
rospack = rospkg.RosPack()
param_path = os.path.join(rospack.get_path("beetle"), "config", "BeetleNMPCFull.yaml")
with open(param_path, "r") as f:
    param_dict = yaml.load(f, Loader=yaml.FullLoader)
nmpc_params = param_dict["controller"]["nmpc"]

class MPCXUPubNode:
        def __init__(self, robot_name: str, file: str) -> None:
                self.robot_name = robot_name
                self.node_name = "mpc_xu_pub_node"
                rospy.init_node(self.node_name, anonymous=False)
                self.namespace = rospy.get_namespace().rstrip("/")
                
                # Pubfd
                self.ref_xu_msg = PredXU()
                # Initialize dimensions if not already done
                if len(self.ref_xu_msg.x.layout.dim) < 2:
                        self.ref_xu_msg.x.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
                if len(self.ref_xu_msg.u.layout.dim) < 2:
                        self.ref_xu_msg.u.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]

                self.pub_ref_xu = rospy.Publisher(f"/{robot_name}/set_ref_x_u", PredXU, queue_size=5)
        
                # nmpc and robot related
                self.N_nmpc = int(nmpc_params["T_pred"] / nmpc_params["T_integ"])
                self.t_dis_mpc = nmpc_params["T_integ"]
                self.t_pred_mpc = nmpc_params["T_pred"]
                self.nx = 23
                self.nu = 8

                # traj
                # Load the CSV file as a numpy array
                current_path = os.path.abspath(os.path.dirname(__file__))
                file_path = current_path + '/'+file
                self.scvx_traj = np.loadtxt(file_path, delimiter=',')
                self.x_traj = self.scvx_traj[0:19, :]
                self.u_traj = self.scvx_traj[19:28, :]

                t_servo = 0.085883
                ac = self.u_traj[4:8, :]*t_servo+self.x_traj[13:17, :]
                self.u_traj[4:8, :] = ac

                # self.x_traj[-2, :] = (self.x_traj[-2, :]/self.x_traj[-2, -1])*8.0
                # self.u_traj[4:8, :] = 0.0

                # Debugging
                # self.x_traj = np.zeros((19, 2))
                # self.u_traj = np.zeros((9, 2))
                # self.x_traj[:, 0] = np.array([0., 0., 1.,
                #                                 0., 0., 0.,
                #                                 1.0, 0., 0., 0.,
                #                                 0., 0., 0.,
                #                                 0., 0., 0., 0.,
                #                                 0., 0.])
                # self.x_traj[:, 1] = np.array([10., 10., 1.,
                #                                 0., 0., 0.,
                #                                 1.0, 0., 0., 0.,
                #                                 0., 0., 0.,
                #                                 0., 0., 0., 0.,
                #                                 10., 0.])
                # self.u_traj[:, 0] = np.array([7.2696, 7.2696, 7.2696, 7.2696, 0., 0., 0., 0., 0.])
                # self.u_traj[:, 1] = np.array([7.2696, 7.2696, 7.2696, 7.2696, 0., 0., 0., 0., 0.])

                self.start_time = rospy.Time.now().to_sec()
                rospy.loginfo(f"{self.namespace}/{self.node_name}: Initialized!")

                # Timer
                self.ts_pt_pub = 0.02  # [s]  ~50Hz
                self.tmr_pt_pub = rospy.Timer(rospy.Duration.from_sec(self.ts_pt_pub), self.callback_tmr_pt_pub)
                rospy.loginfo(f"{self.namespace}/{self.node_name}: Timer started!")

        def callback_tmr_pt_pub(self, timer: rospy.timer.TimerEvent):
                # 1. check if the frequency is too slow
                if timer.last_duration is not None and self.ts_pt_pub < timer.last_duration:
                        rospy.logwarn(
                                f"{self.namespace}: Control is too slow!"
                                f"ts_pt_pub: {self.ts_pt_pub * 1000:.3f} ms < ts_one_round: {timer.last_duration * 1000:.3f} ms"
                        )

                # 2. prepare the message
                
                current_time = rospy.Time.now().to_sec()
                t_has_started = current_time - self.start_time
                if t_has_started >= 1.0:
                        t_has_started -= 1.0
                        t_nodes = np.linspace(0, self.t_pred_mpc, self.N_nmpc + 1)
                        t_nodes += t_has_started

                        # interpolate the reference trajectory
                        x_traj = np.zeros((self.N_nmpc + 1, self.nx))
                        u_traj = np.zeros((self.N_nmpc, self.nu))

                        for i in range(self.nx-6):
                                x_traj[:, i] = np.interp(t_nodes, self.x_traj[-2, :], self.x_traj[i, :])
                        for i in range(self.nu):
                                u_traj[:, i] = np.interp(t_nodes[:-1], self.x_traj[-2, :], self.u_traj[i, :])

                        self.ref_xu_msg.header.stamp = rospy.Time.now()
                        self.ref_xu_msg.header.frame_id = "map"
                        self.ref_xu_msg.x.layout.dim[1].stride = self.nx
                        self.ref_xu_msg.u.layout.dim[1].stride = self.nu
                        self.ref_xu_msg.x.layout.dim[0].size = self.N_nmpc + 1
                        self.ref_xu_msg.u.layout.dim[0].size = self.N_nmpc
                        self.ref_xu_msg.x.data = x_traj.flatten().tolist()
                        self.ref_xu_msg.u.data = u_traj.flatten().tolist()
                        self.pub_ref_xu.publish(self.ref_xu_msg)
                
                # if t_has_started <= self.x_traj[-2, -1]: # trajectory not finished
                #         t_nodes = np.linspace(0, self.t_pred_mpc, self.N_nmpc + 1)
                #         t_nodes += t_has_started

                #         # interpolate the reference trajectory
                #         x_traj = np.zeros((self.N_nmpc + 1, self.nx))
                #         u_traj = np.zeros((self.N_nmpc, self.nu))

                #         for i in range(self.nx-6):
                #                 x_traj[:, i] = np.interp(t_nodes, self.x_traj[-2, :], self.x_traj[i, :])
                #         for i in range(self.nu):
                #                 u_traj[:, i] = np.interp(t_nodes[:-1], self.x_traj[-2, :], self.u_traj[i, :])

                #         self.ref_xu_msg.header.stamp = rospy.Time.now()
                #         self.ref_xu_msg.header.frame_id = "map"
                #         self.ref_xu_msg.x.layout.dim[1].stride = self.nx
                #         self.ref_xu_msg.u.layout.dim[1].stride = self.nu
                #         self.ref_xu_msg.x.layout.dim[0].size = self.N_nmpc + 1
                #         self.ref_xu_msg.u.layout.dim[0].size = self.N_nmpc
                #         self.ref_xu_msg.x.data = x_traj.flatten().tolist()
                #         self.ref_xu_msg.u.data = u_traj.flatten().tolist()
                #         self.pub_ref_xu.publish(self.ref_xu_msg)

                # else: # trajectory finished
                #         rospy.loginfo(f"{self.namespace}/{self.node_name}: Trajectory finished!")
                #         rospy.signal_shutdown("Trajectory finished!")
                #         return
                        
if __name__ == "__main__":
        parser = argparse.ArgumentParser(description="MPC Point Trajectory Publisher Node")
        parser.add_argument("robot_name", type=str, help="Robot name, e.g., beetle1, gimbalrotors")
        parser.add_argument("-f", "--file", type=str, default="scvx_traj.csv", help="trajectory file")

        args = parser.parse_args()

        try:
                node = MPCXUPubNode(args.robot_name, args.file)
                rospy.spin()
        except rospy.ROSInterruptException:
                pass
                
               

                

