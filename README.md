# WAM-V Autonomous Vessel Navigation
This repository, alongside the repository managed by @uasif13, is responsible for the autonomous navigation a WAM-V 16 ASV. The current resources here allow developers to collect camera images through ROS and Gazebo then determine the location of buoys. All work here has been tested with the Virtual RobotX simulation environment with the eventual goal of traversing between sets of buoys and then eventually reaching full autonomous movement and obstacle avoidance.

## Getting Started
These instructions should allow you to get the project up and running on your system.

### Prerequisites
Install ROS and the Virtual RobotX (VRX) environment using either of the installation methods that VRX offers. Note that if you install the VRX Docker image that you will still need a copy of ROS on your local machine in order to interact with Gazebo on Docker.
  
In addition, users should install the following Python packages (versions are the latest tested):
* OpenCV (version = 3.2.0)
* NumPy (version = 1.16.4)
* Matplotlib (version = 3.1.0)
* Scikit-learn (version = 0.21.2)

All of the code is written using Python 2 in a Jupyter notebook and as such both packages should be installed. ROS Package equivalents will be implemented when a final version of the notebooks are prepared.

### Install Repository
Pull the repository, no additional setup is required

### Run most recent working version
Open an instance of the Docker image and setup the ROS_MASTER_URI and ROS_IP to your liking. From there run the following command in an instance of the Docker image:

`$ roslaunch vrx_gazebo vrx.launch`

`$ roslaunch vrx_gazebo usv_keydrive.launch`

If you have a controller or joystick that you would like to use, plug it in to your computer and launch usv_joydrive.launch instead of usv_keydrive.launch

And then in separate terminals on the host machine run the following commands (after setting up the ROS_MASTER_URI and ROS_IP)

`$ ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc`

`$ rosrun rqt_reconfigure rqt_reconfigure`

With those four commands, the WAM-V should be in a customizable state that will allow you to run the latest code. To test that everything is working properly, `$ rostopic list` can be run but be warned that with stereo_image_proc running there will be nearly 150 topics being published. With the system set up properly, run Asif_pool_live.ipynb in Jupyter Notebook ***update this to the latest version of the code*** and two windows should appear, a disparity map and the filtered image from the left lens of the front camera.

\*\*\*to be continued\*\*\*
  
This material is based upon work funded by the U.S. Department of Homeland Security under Cooperative Agreement No. 2014-ST-061-ML0001

