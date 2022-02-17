# SLAM-backend-tutorial  

You could see the Video lecture on [Youtube](https://youtu.be/FhwFyA0NQkE).  

The original code, slides, and notes are available in [Google drive](https://drive.google.com/drive/folders/1_MLRQMvRuXvyxVl6514M20zcFY4-O4pI?usp=sharing).  

I test this repository on Ubuntu 18.04.  

### Contents  
- [How to make same enviornment with Docker.](#how-to-make-same-enviornment-with-docker)  
- [How to execute this code.](#how-to-execute-this-code)  
- [Result](#result)  

## How to make same enviornment with Docker.  

First you have to install [Docker](https://www.docker.com/).  

**1. Just clone this repository and change the directory path.**  
```
git clone https://github.com/Taeyoung96/SLAM-backend-tutorial.git  
cd SLAM-backend-tutorial
```  

**2. Build the SLAM docker image.**  
This docker image includes ubuntu 18.04, ros melodic, [ceras-solver](http://ceres-solver.org/), and [gtsam](https://gtsam.org/).  
```
docker build -t tutorial-week-2020/slam:latest .
```  

When you are finished, check the image.  
```
docker images
```
You can check that the image is created as below in your terminal, the IMAGE ID and CREATED TIME may be different.  
```
REPOSITORY                TAG        IMAGE ID       CREATED         SIZE
tutorial-week-2020/slam   latest     18d7778e6a07   22 hours ago    1.46GB
ros                       melodic    2529d0ef4064   2 weeks ago     1.28GB
```  

**3. Start a Docker container while sharing the container's volume directory with a local directory.**  
```
docker run -v ${CODE_DIR}/slam_tutorial:/root/slam_ws/src/slam_tutorial -n slam_tutorial -it tutorial-week-2020/slam:latest  
```

In my case, ${CODE_DIR} is `/home/taeyoung/Desktop/SLAM-backend-tutorial/`, it may depend on where you clone the repository.  
For example,  
```
docker run -v /home/taeyoung/Desktop/SLAM-backend-tutorial/slam_tutorial:/root/slam_ws/src/slam_tutorial -n slam_tutorial -it tutorial-week-2020/slam:latest  
```
When the container creation is completed, the terminal path is changed and you can see that you are connected to the docker container.  
```
root@bf318ac3a3d7:~# 
```  
Check the directory in the container and the local directory are shared.  
```
root@bf318ac3a3d7:~# cd slam_ws/
root@bf318ac3a3d7:~/slam_ws# cd src
root@bf318ac3a3d7:~/slam_ws/src# ls
CMakeLists.txt  slam_tutorial
```  
## How to execute this code.  

**1. Just build the code with `catkin_make`**  

```
root@bf318ac3a3d7:~/slam_ws# catkin_make
```
After the build is complete, you can see the output as below and a build folder and a devel folder are created.    
```
Scanning dependencies of target ceres_spg
Scanning dependencies of target gtsam_spg
[ 25%] Building CXX object slam_tutorial/CMakeFiles/ceres_spg.dir/src/ceres_spg.cpp.o
[ 50%] Building CXX object slam_tutorial/CMakeFiles/gtsam_spg.dir/src/gtsam_spg.cpp.o
[ 75%] Linking CXX executable /root/slam_ws/devel/lib/slam_tutorial/ceres_spg
[ 75%] Built target ceres_spg
[100%] Linking CXX executable /root/slam_ws/devel/lib/slam_tutorial/gtsam_spg
[100%] Built target gtsam_spg
```  

**2. Execute the code with `rosrun` command.**  

You could execute two different node. One is for Ceres-solver example, the other is for gtsam example.  

If you want to run the ceres-solver example node, you enter the following command.  
```
rosrun slam_tutorial ceres_spg
```

If you want to run the gtsam example node, you enter the following command.  
```
rosrun slam_tutorial gtsam_spg
```

## Result  






