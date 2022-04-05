This repository provides optimal and robust control algorithms for indoors autonomous navigation of DJI M100 using UWB wireless-based localization.
The filtering and estimation is done by: 
1. Kalman filter
2. Maximum Correntropy Criterion Kalman filter (MCC-KF)

Prerequisites:
1. Ubuntu 16.04
2. Install ROS Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu)
3. Create a ROS workspace (http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
4. Install the DJI OSDK and ROS nodes (https://developer.dji.com/onboard-sdk/documentation/sample-doc/sample-setup.html#ros-onboard-computer)

**cite** This project contains the implementation for our [paper](https://research.aalto.fi/files/51270426/ELEC_Makridis_Charalambous_Towards_Robust_Onboard_IWCMC2020_acceptedauthormanuscript.pdf).  If you find this code useful in your research, please consider citing:

```bibtex
@inproceedings{makridis2020towards,
  title={Towards robust onboard control for quadrotors via ultra-wideband-based localization},
  author={Makridis, Evagoras and Charalambous, Themistoklis},
  booktitle={2020 International Wireless Communications and Mobile Computing (IWCMC)},
  pages={1630--1635},
  year={2020},
  organization={IEEE}
}
```
