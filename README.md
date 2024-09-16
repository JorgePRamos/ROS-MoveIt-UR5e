# ROS MoveIt UR5e
This repository is aimed to provided a needed starting point for any project which uses the following configuration:
- [UR5e](https://www.universal-robots.com/products/ur5-robot/)
- [Robotiq 2 finger gripper 2F-85](https://robotiq.com/products/adaptive-grippers#Two-Finger-Gripper)
- [ROS1 Noetic](https://wiki.ros.org/Distributions)
- [MoveIt](https://moveit.ai/)
- [Ubuntu Focal Fossa](https://releases.ubuntu.com/focal/)


![alt text](/Docs/Media/sim_cobot.png "Rviz cobot set up")



## Quick start
If you have an existing ROS + MoveIt environment and want to test out the provided configuration, follow this two simple steps:
- Clone this repository in your machine and make sure to checkout the main branch.
- Source the provided `full_start.sh` file by running:

```bash
source full_start.sh
```
## Step by step guide
Find [here](/Docs/UR5e_backlog_g_and_guide.pdf) an extensive step by step guide in development backlog format, which will show you every detail needed from setting up the UR5e and ROS machine to MoveIt configuration and advance adjustments and gripper use.


## Author's note
I hold this repository really close to my heart, since it represents my take on filling an information gap I encountered while first diving into the ROS world. I have included everything I would have found necessary to not only understand the process but dive down into the unimportant details, which some times are skipped over but can give many headaches for the inexperienced.

I hope you find this repository helpful and if so a star is always appreciated ;)


