The following note includes notes about my project to integrate
cartographer into the bwi codebase.


Table of Contents:
    1) Installation
    2) Running the Demos
    3) Running in BWI Simulation (For V2)



1) Installation

    What to do BEFORE you begin!
        Make sure that you sudo priviliges on your commputer, if you do
        not you will have to request them.
        The reccomended system specs for running Cartographer are:
            1) 64 bit CPU
            2) 16 GB RAM
            3) Ubuntu 14.04 (Trusty) or 16.04 (Xenial)
            4) gcc version 4.8.4 and 5.4.0
        --This information came from https://google-cartographer.readthedocs.io/en/latest/index.html#system-requirements

    Now we may begin the actual installation:

2) Running the Demos
    
    When you compile this code you must to catkin_make_isolated and NOT
    catkin_make. The difference between the two is that catkin_make does all the
    linking with catkin packages. The command catkin_make_isolated is used for 
    "non-homogeneous workspaces". This means that not all the packages in this 
    workspace are catkin_packages. Part of the reason we use 
    catkin_make_isolated here is becaus cartographer was frst made as a C++ 
    library. It was then adapted to integrate with ROS, but it kept some of 
    those originally packages. 


