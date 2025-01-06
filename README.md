# traj_loader_tilt_mt

This is for publishing trajectory for tilting quadrotors.

Pub trajectory node:
<code>rosrun traj_loader_tilt_mt pub_xu_traj.py beetle1 -f scvx_traj.csv</code>. 
***Note***: change the file name accordingly.
Start tracking command:
<code>rostopic pub -1 /beetle1/trajectory_tracking std_msgs/Bool "{data: True}"</code>

