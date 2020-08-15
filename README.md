# Intelligent Picking

A brief Overview of our approach could be found in this video - **[VIDEO SUBMISSION](https://www.youtube.com/watch?v=IlN6-pN3mRA&feature=youtu.be)**

# Introduction :
The problem of pick and place has been one of the actively studied area and a canonical problem in robotics. The Amazon Robotics Challenge (ARC) has a rich tradition for the fabrication of highly robust and competitive warehouse robots that do classify and segregate objects apart from just pick and place.The advent of Deep Reinforcement Learning as a reliable alternate for learning robot controllers has greatly increased the dexterity and robustness of these arms.The given problem statement of Flipkart Grid 2.0, is quite unique, unparalleled, challenging, and demands a great amount of customization and design improvements in terms of both hardware and software. The enormous dimension of the arena and the relatively heavier payload entirely eliminates the possibility of using any pre-existing methodologies. Also fabricating a robot from scratch at the given price point makes the challenge event the more exciting.Thus, we are sharing a solution for the above task, with all our experiments and results which according to the best of our knowledge the most cost-efficient, simplistic yet robust approach

# Hardware Design
<p align="center">
   <img width="250" height="250" src="https://github.com/RaISy-Net/Intelligent_picking/blob/master/ReadmeImages/ro.png">
   <img width="250" height="250" src="https://github.com/RaISy-Net/Intelligent_picking/blob/master/ReadmeImages/ws.png">
   <img width="250" height="250" src="https://github.com/RaISy-Net/Intelligent_picking/blob/master/ReadmeImages/ca.png">
</p>

* Our robot is greatly inspired by cartman, owing to its cost-efficient cartesian design which could cover the entire work area in a stable fashion.
* The generic 6 DOF robot arm, requires high torque motors at every joint to support the
payload at the end effector whose costs are around INR 10,000 per unit. However the torque to be applied per joint is drastically decreased due to our design and thus we are unaffected by the above limitation.

# Software Pipeline

<p align="center">
   <img width="250" height="250" src="https://github.com/RaISy-Net/Intelligent_picking/blob/master/ReadmeImages/sp.png">


<p align="center">
   <img width="250" height="250" src="https://github.com/RaISy-Net/Intelligent_picking/blob/master/ReadmeImages/OD.jpeg.jpg">
   <img width="350" height="250" src="https://github.com/RaISy-Net/Intelligent_picking/blob/master/ReadmeImages/Ge.png">
   <img width="250" height="250" src="https://github.com/RaISy-Net/Intelligent_picking/blob/master/ReadmeImages/ge2.png">
</p>

For a more deatailed explanation of our work, check out our Phase 2 Report submission - [Repot.pdf](https://github.com/RaISy-Net/Intelligent_picking/blob/master/Phase_2_submission/Docs/X%20Ash%20A%20-%2012_IIT%20(BHU)%20Varanasi_Intelligent_Picking_-_Round_3.pdf)


# Ongoing Work:

* Having validated our solution in **Pybullet** Simulator, we are now moving on to build a real world prototype that closely resembles our idea within the given budget of **INR 50,000**

* We are actively working in addressing the the problems like **simulation to reality transfer** of our approach and customization of the pipeline for our pipeline for the fabricated hardware.

Any contributions/ suggestions are most welcome. 

# References:
## Papers & Blogs
* [Robotic Pick-and-Place of Novel Objects
in Clutter with Multi-Affordance Grasping
and Cross-Domain Image Matching](https://arxiv.org/pdf/1710.01330.pdf)
* [Robotic Grasping of Novel Objects using Vision
](http://pr.cs.cornell.edu/grasping/IJRR_saxena_etal_roboticgraspingofnovelobjects.pdf)
* [Deep Reinforcement Learning for Vision-Based Robotic Grasping:
A Simulated Comparative Evaluation of Off-Policy Methods](https://arxiv.org/pdf/1802.10264v2.pdf)
* [Vision-based Robotic Grasp Detection From Object Localization, Object Pose Estimation To Grasp Estimation: A Review](https://arxiv.org/pdf/1905.06658.pdf)
* [Robotic Grasping in Cluttered Environments-Stanford Videos](https://ai.stanford.edu/~asaxena/clutteredgrasping/)
* [Analysis and Observations from the
First Amazon Picking Challenge
](https://arxiv.org/pdf/1601.05484.pdf)
* [Team Delft’s Robot Winner of the Amazon
Picking Challenge 2016
](https://arxiv.org/pdf/1610.05514.pdf) - Their implementation [GitHub](https://github.com/warehouse-picking-automation-challenges/team_delft)
* [AN ANALYTICAL METHOD TO FIND WORKSPACE OF A ROBOTIC
MANIPULATOR](https://pdfs.semanticscholar.org/e5f8/d98ce96b1dfcce05966bed52a85a215cf0a9.pdf)
* [How I won the Flipkart ML challenge](https://towardsdatascience.com/how-i-won-the-flipkart-ml-challenge-fcf1fcc9e06a)
* [Domain Randomization for Sim2Real Transfer](https://lilianweng.github.io/lil-log/2019/05/05/domain-randomization.html)
* [Amazon picking challenge - Cartman - Robot using 3D printer system](https://arxiv.org/pdf/1709.06283.pdf)
* [Amazon picking challenge - MIT and Princeton](https://vision.princeton.edu/projects/2017/arc/)
* Grasp Prediction on RGB-D images [[Paper](https://arxiv.org/abs/1909.04810)] [[Code](https://github.com/skumra/robotic-grasping)]


## Papers on Grasping and Pose estimation
* [RefineNet for object segmentation](https://arxiv.org/pdf/1611.06612.pdf)
* [Light weight RefineNet for object segmentation from RGB-D images](https://paperswithcode.com/paper/light-weight-refinenet-for-real-time-semantic)
* [Training COCO dataset to master object segmentation - MEDIUM](https://towardsdatascience.com/master-the-coco-dataset-for-semantic-image-segmentation-part-1-of-2-732712631047)
* [DenseFusion: 6D Object Pose Estimation by Iterative Dense Fusion](https://arxiv.org/pdf/1901.04780v1.pdf) - Implimentation [GitHub](https://github.com/j96w/DenseFusion)
* [Using Geometry to Detect Grasp Poses in 3D Point Clouds](http://www.ccs.neu.edu/home/atp/publications/grasp_poses_isrr2015.pdf)
* [Using Geometry to Detect Grasping Points on 3D Unknown Point Cloud](https://rua.ua.es/dspace/bitstream/10045/75568/1/ICINCO_2017_182_CR.pdf)
* [Vision-based Robotic Grasp Detection From Object Localization, Object Pose Estimation To Grasp Estimation: A Review](https://arxiv.org/pdf/1905.06658.pdf) - Survey Paper
* [Efficient Grasping from RGBD Images: Learning using a new Rectangle Representation](http://pr.cs.cornell.edu/grasping/jiang_rectanglerepresentation_fastgrasping.pdf)



## Other GitHub links
* [MIT-Princeton Vision Toolbox for the APC 2016](https://github.com/andyzeng/apc-vision-toolbox)
* [Motoman robots files](https://github.com/ros-industrial/motoman)
* [Kawasaki Robotics files](https://github.com/Kawasaki-Robotics/khi_robot)

# Tools
* [Robot arm calculator](https://www.societyofrobots.com/robot_arm_calculator.shtml)
* [igus robo link designer](https://robolink-designer.igus.tools/construction)

# Datasets
* [CMU grocery dataset](http://www.cs.cmu.edu/~ehsiao/datasets.html)
