
Visual Odometry using RGB-D Camera Tutorial
==============================================
**Author**: **Niranjan Sujay**


**Step 1: Introduction** 

Let's delve into Visual Odometry, a key element in computer vision and robotics, guided by Niranjan Sujay. This tutorial focuses on enabling robots to comprehend their surroundings by analyzing sequentially captured images. Visual Odometry plays a crucial role in helping robots understand their movements through onboard cameras.

   - Explore the mechanics of Visual Odometry, a fundamental component in robotic navigation and perception. Imagine a robot in an unknown environment figuring out its location merely by observing its surroundings. In this tutorial, we'll break down the steps and employ techniques to make Visual Odometry work.

   - In the realm of computer vision, Visual Odometry takes center stage, enabling robots to estimate their position and orientation by tracking features across consecutive images. From reading sensor data to processing color and depth images and implementing optical flow techniques, we cover each step in detail. The ultimate objective is to predict the robot's path and pose—an essential aspect of autonomous navigation.

The tutorial will guide you through the entire process, from reading sensors to manipulating images and employing clever mathematics to track points over time, showcasing the practical utility of Visual Odometry.

For hands-on practice, you can check out the **RoboticsAcademy Visual Odometry Exercise**
learning at 
   - `<https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/visual_odometry>`__.  
   - `<https://link.springer.com/chapter/10.1007/978-3-319-29363-9_14>`.
 It provides a practical platform for applying the concepts covered here. 

This tutorial is for anyone curious about making robots see and understand the world better. Let's jump in and explore Visual Odometry together!

**Step 2: Prerequisites** 
- Before starting this tutorial, you should have a basic understanding of Python, OpenCV, and linear algebra.

**Step 3: Tools and Libraries** 
- We will be using Python and OpenCV for this tutorial. If you haven’t installed them yet, you can follow these guides: 
    - Python Installation Guide - `https://www.python.org/downloads/`, 
    - OpenCV Installation Guide - `https://opencv.org/get-started/`.

Environment lookup
 
- Robotics Academy supports Linux Ubuntu 20.04 and is based in ROS-Noetic for this excersie

Anlogside the additional installation based on the guide from JdeRobot academy are - 

- Installation of these packages before proceeding.
    - pyqtgraph — sudo pip install pyqtgraph

    - configparser — sudo pip install configparser

To launch the exercise:

  -  Download the rosbag file from here. `https://gsyc.urjc.es/jmplaza/slam/rgbd_dataset_freiburg2_pioneer_slam_truncated.bag`

  -  gitclone - `https://github.com/JdeRobot/RoboticsAcademy.git` 

   - Navigate to the Visual odmetry excersie and then place the rosbag file in the same directory as of this exercise and replace the name of the rosbag file in the ‘visual_odometry.cfg’ or mention the full path of the rosbag file.

   - Execute the exercise with GUI : python visual_odom.py

**Step 4: Theoretical Explanation** 

1. **Overview of Visual Odometry:**
   - It's a path planning technique employed by robots to estimate the motion of a camera in real-time using sequential images.
   - The concept traces back to the 1980s with Moravec's work at Stanford.

2. **Fundamental Algorithm:**
   - The algorithm involves:
     - Identifying features of interest in each frame.
     - Matching features across frames.
     - Estimating the rigid body transformation that best aligns these features over time.

     ![image.png](attachment:image.png)

3. **Advancements in Visual Odometry:**
   - Modern algorithms use feature detectors like Harris corners and FAST features.
   - These algorithms are known for their speed and resilience against minor viewpoint changes.
   - Methods for robustly matching features across frames have evolved, including RANSAC-based techniques and graph-based consistency algorithms.


        ![image-2.png](attachment:image-2.png)


4. **Evolution of Motion Estimation Techniques:**
   - Techniques have evolved from directly minimizing Euclidean distance between matched features to minimizing pixel reprojection error.
   - Bundle adjustment, when computational resources allow, helps in reducing integrated drift.

5. **Challenges with Visual Odometry:**
   - Visual odometry often suffers from unbounded global drift, where errors accumulate over time without correction, leading to significant deviations from the true trajectory.

6. **Integration with SLAM:**
   - To address unbounded global drift, visual odometry is integrated with simultaneous localization and mapping (SLAM) algorithms.
   - SLAM algorithms employ loop closure techniques to detect revisited locations.

7. **Recent Trends in Visual SLAM:**
   - Recent visual SLAM methods rely heavily on fast image matching techniques for loop closure.
   - Loop closure involves detecting and correcting errors that arise when revisiting previously visited locations.
   - A common approach is constructing a pose graph representing spatial relationships between robot positions and environmental features, creating constraints that link previous poses.

Theory on Visual Odometry & approach for this exercise: 

To enhance the understanding of the core algorithm with additional mathematical expressions and detailed explanations, let's further dissect the components related to feature detection, optical flow, 3D point cloud creation, and motion estimation.

### Feature Detection Using FAST Algorithm

The intensity of a corner candidate pixel *P* is denoted as *Ip*. For *P* to be considered a corner, there must exist *N* contiguous pixels *p1, p2, ..., pN* in its circle of radius *r* such that:

![image-13.png](attachment:image-13.png)

where \(T\) is the intensity threshold.

### Lucas-Kanade Optical Flow
- For a pixel *I*(*x*, *y*, *t*) moving by displacement (*dx*, *dy*) between two frames at time *t* and *t*+*dt*, the Lucas-Kanade method assumes brightness constancy:

![image.png](attachment:image.png)

- Expanding *I*(*x* + *dx*, *y* + *dy*, *t* + *dt*) using Taylor series and assuming small movements gives:

![image-2.png](attachment:image-2.png)

where *u*=*dx*/*dt* and *v*=*dy*/*dt* are the optical flow components in *x* and *y* directions, and *I*_x, *I*_y, and *I*_t are the partial derivatives of *I* with respect to *x*, *y*, and *t* respectively.

### 3D Point Cloud Creation

- The transformation from pixel coordinates (*x*, *y*) and depth *Z* to 3D coordinates (*X*, *Y*, *Z*) in camera frame is given by:

\ *X* = (*x* - *c*_x) * {*Z*}/{*f*_x} \

\ *Y* = (*y* - *c*_y) * {*Z*}/{*f*_y} \

\ *Z* = depth value / scale factor 

where \(*f*_x, *f*_y\) are the camera's focal lengths along \(*x*\) and \(*y*\) axes, and \(*c*_x, *c*_y\) are the coordinates of the principal point.

### Motion Estimation Between Consecutive 3D Point Clouds

- Given two sets of points *P* and *Q* representing the same set of points in the world but captured from two consecutive frames, the goal is to find the rigid transformation (rotation *R* and translation *t*) that best aligns *P* to *Q*:

The optimal rotation \(R\) and translation \(t\) minimize the sum of squared differences:

![image-3.png](attachment:image-3.png)

This problem is often solved using Singular Value Decomposition (SVD) on the cross-covariance matrix \(H = PQ^T\), where \(P\) and \(Q\) are centered (mean subtracted) versions of the original point sets.

1. Compute centroids ![image-12.png](attachment:image-12.png) and ![image-5.png](attachment:image-5.png) of \(P\) and \(Q\).
2. Center \(P\) and \(Q\) by subtracting \(\bar{P}\) and \(\bar{Q}\) respectively.
3. Compute ![image-6.png](attachment:image-6.png).
4. Decompose \(H\) using SVD: ![image-7.png](attachment:image-7.png).
5. The rotation is given by ![image-8.png](attachment:image-8.png), and the translation is ![image-9.png](attachment:image-9.png).

### Concatenating Rotation and Translation Information

- The homogeneous transformation matrix *T* that encapsulates both rotation *R* and translation *t* is:

![image-10.png](attachment:image-10.png)

- Updating the cumulative transformation *C*_{cum} with a new transformation *T* is done by matrix multiplication:

![image-11.png](attachment:image-11.png)

This operation applies the latest motion estimate to the cumulative path, effectively tracking the device's trajectory over time.

**Code to Mathematical Concept Mapping:**

- **Feature Detection (FAST)**: The FAST algorithm's implementation in `cv2.FastFeatureDetector_create()`. 
- **Optical Flow (Lucas-Kanade)**: `cv2.calcOpticalFlowPyrLK(self.gray_image_prev, gray_image, self.p0, None, **self.lk_params)` embodies the optical flow estimation, translating the brightness constancy and spatial gradient concepts into motion vectors.
- **3D Point Cloud Creation**: The snippet where `Z`, `X`, and `Y` are computed from pixel coordinates and depth illustrates the projection from 2D image space to 3D world space.
- **Motion Estimation**: Computation involves creating and manipulating 3D point clouds and potentially applying transformations to estimate motion.

These mathematical models and their code implementations are foundational to computer vision and robotics, enabling sophisticated tasks such as navigation, mapping, and interaction with the environment.

**Step 5: Code Walkthrough** Now, Let’s break down the code into smaller parts and explain each part in detail.

According to the exercise, we must edit the MyAlgorithm.py file and insert the algorithm logic into it.

import threading
import time
import math
from turtle import color
import rosbag
import cv2
import numpy as np
from datetime import datetime
import random
import numpy.linalg
