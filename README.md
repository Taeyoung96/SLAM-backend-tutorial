# SLAM-backend-tutorial  

You could see the Video lecture on [Youtube](https://youtu.be/FhwFyA0NQkE).  

The original code, slides, and notes are available in [Google drive](https://drive.google.com/drive/folders/1_MLRQMvRuXvyxVl6514M20zcFY4-O4pI?usp=sharing).  

`slam_tutorial/` folder involves my personal solution, and  `slam_tutorial_solution/` involves given solution.  
The contents are almost similar.  

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

When you following above command, you share the `slam_tutorial/` folder.  
If you want to change with `slam_tutorial_solution/`,  
just change the directory `slam_tutorial/` to `slam_tutorial_solution/` when you run the above command.  

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

- Result output when you run ceres-solver example node  

You could visualize `.csv` files, with [Matlab](https://kr.mathworks.com/products/matlab.html).  
I got different results because I designed the error function differently.  
<p align="center"><img src="/result/error_function.gif" width = "150" ></p>  

- Initial pose  

<p align="center"><img src="/result/initial_pose.png" width = "500" ></p>  

- Optimized_pose with my personal solution  

<p align="center"><img src="/result/optimized_pose.png" width = "300" ></p>  

- Optimized_pose with given soltion  

<p align="center"><img src="/result/optimzied_pose_solution.png" width = "300" ></p>  

```
Solver Summary (v 1.14.0-eigen-(3.3.4)-lapack-suitesparse-(5.1.2)-cxsparse-(3.1.9)-eigensparse-openmp-no_tbb)

                                     Original                  Reduced
Parameter blocks                          552                      550
Parameters                               1932                     1925
Effective parameters                     1656                     1650
Residual blocks                          1073                     1073
Residuals                                6438                     6438

Minimizer                        TRUST_REGION

Sparse linear algebra library    SUITE_SPARSE
Trust region strategy     LEVENBERG_MARQUARDT

                                        Given                     Used
Linear solver          SPARSE_NORMAL_CHOLESKY   SPARSE_NORMAL_CHOLESKY
Threads                                     1                        1
Linear solver ordering              AUTOMATIC                      550

Cost:
Initial                          2.716861e+02
Final                            5.204096e+00
Change                           2.664820e+02

Minimizer iterations                      161
Successful steps                          149
Unsuccessful steps                         12

Time (in seconds):
Preprocessor                         0.000880

  Residual only evaluation           0.026268 (161)
  Jacobian & residual evaluation     0.265551 (149)
  Linear solver                      0.838873 (161)
Minimizer                            1.193949

Postprocessor                        0.000041
Total                                1.194870

Termination:                      CONVERGENCE (Function tolerance reached. |cost_change|/cost: 8.268293e-07 <= 1.000000e-06)

```


- Result output when you run gtsam example node  

```
Factor Graph:
size: 6

Factor 0: PriorFactor on x1
  prior mean:  (0, 0, 0)
  noise model: diagonal sigmas[1; 1; 0.1];

Factor 1: BetweenFactor(x1,x2)
  measured:  (5, 0, 0)
  noise model: diagonal sigmas[0.5; 0.5; 0.1];

Factor 2: BetweenFactor(x2,x3)
  measured:  (5, 0, -1.57079633)
  noise model: diagonal sigmas[0.5; 0.5; 0.1];

Factor 3: BetweenFactor(x3,x4)
  measured:  (5, 0, -1.57079633)
  noise model: diagonal sigmas[0.5; 0.5; 0.1];

Factor 4: BetweenFactor(x4,x5)
  measured:  (5, 0, -1.57079633)
  noise model: diagonal sigmas[0.5; 0.5; 0.1];

Factor 5: BetweenFactor(x5,x2)
  measured:  (5, 0, -1.57079633)
  noise model: diagonal sigmas[0.5; 0.5; 0.1];


Initial Values:

Values with 5 values:
Value x1: (gtsam::Pose2)
(0.2, -0.3, 0.2)

Value x2: (gtsam::Pose2)
(5.1, 0.3, -0.1)

Value x3: (gtsam::Pose2)
(9.9, -0.1, -1.77079633)

Value x4: (gtsam::Pose2)
(10.2, -5, -3.04159265)

Value x5: (gtsam::Pose2)
(5.1, -5.1, 1.47079633)

Initial error: 18.510326
newError: 0.122934358
errorThreshold: 0.122934358 > 0
absoluteDecrease: 18.3873916591 >= 1e-05
relativeDecrease: 0.993358606565 >= 1e-05
newError: 8.85829965247e-06
errorThreshold: 8.85829965247e-06 > 0
absoluteDecrease: 0.12292549938 >= 1e-05
relativeDecrease: 0.999927942848 >= 1e-05
newError: 3.68234840295e-15
errorThreshold: 3.68234840295e-15 > 0
absoluteDecrease: 8.85829964879e-06 < 1e-05
relativeDecrease: 0.999999999584 >= 1e-05
converged
errorThreshold: 3.68234840295e-15 <? 0
absoluteDecrease: 8.85829964879e-06 <? 1e-05
relativeDecrease: 0.999999999584 <? 1e-05
iterations: 3 >? 100
Final Result:

Values with 5 values:
Value x1: (gtsam::Pose2)
(-1.27918496267e-18, 1.66618678917e-19, -3.14163502147e-20)

Value x2: (gtsam::Pose2)
(5, 5.11915975726e-20, -7.11636343752e-20)

Value x3: (gtsam::Pose2)
(10.0000000015, -4.40576423166e-09, -1.5707963267)

Value x4: (gtsam::Pose2)
(10.0000000114, -5.00000003139, 3.14159265352)

Value x5: (gtsam::Pose2)
(4.99999999784, -5.00000000264, 1.57079632663)

x1 covariance:
                 1 -1.71241483502e-17 -5.65095102146e-17
-1.71241483502e-17                  1   1.7763568394e-16
-5.65095102146e-17   1.7763568394e-16               0.01
x2 covariance:
              1.25 -3.79952940193e-16 -1.40205920175e-16
-3.79952940193e-16                1.5               0.05
-1.40205920175e-16               0.05               0.02
x3 covariance:
     2.70000000047 -8.21536053911e-10    -0.155000000029
-8.21536047854e-10      1.45000000006  -0.00499999990562
   -0.155000000029  -0.00499999990562    0.0264999999985
x4 covariance:
   2.1125000074  0.800000006448 -0.120000000784
 0.800000006448   2.80000000387 -0.170000000296
-0.120000000784 -0.170000000296 0.0279999999952
x5 covariance:
  1.69999999991 -0.224999999954 0.0449999999659
-0.224999999954   2.06250000049 -0.127500000037
0.0449999999659 -0.127500000037 0.0264999999968

```






