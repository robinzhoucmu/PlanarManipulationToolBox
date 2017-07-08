# A package for planar manipulation research. 
This is a simulation, planning and control package for planar manipulation, with an emphasis on fidelity of sliding motion. 

See tutorial folders for sample code usage. 

The core functionalities include:
1) Frictional distribution identification for patch contacts. The package supports ellipsoid (convex quadratic) and 4th order convex polynomial representation of the limit surface model. The optimization routine requires CVX package. 
See reference https://arxiv.org/pdf/1602.06056v3.pdf for more details. 

2) Quasi-static simulation of position-controlled rigid manipulator-object interaction (including grasping and pushing) given geometry description. The simulator is a event-driven system which detects contact (single or multiple) and solves a formulated linear complementarity problem. Once jamming or grasping occurs, the simulation will stop. So you don't have to worry about further penetration. The contact modes during the interaction are logged.
See reference https://arxiv.org/pdf/1705.10664.pdf for more details.


3) Uncertainty propogation: Given a set of initial poses of the object and an action of the manipulator as represented by position-controlled configuration space trajectory, the simulator will progagate the uncertainty.  We use this to synthesis grasping strategies under uncertainty: generate a sequence of squeeze to completely shrink the uncertainty to a known singleton. See reference http://ri.cmu.edu/wp-content/uploads/2017/06/main_revised_v2.pdf for detail.


4) Planning pushing actions: Given an initial pose the object, a desired final pose of the object, a chosen pushing point in local frame and the coefficient of contact friction. The planar returns the time-optimal reference trajectory to push the object from start to goal pose. The planar uses differential flatness property of the pusher-slider system for sticking contact and efficiently gives analytical solutions. 

5) Helper files include plotting tools and SE(2) algebra.

External packages:
Polygonal geometry:
https://www.mathworks.com/matlabcentral/fileexchange/7844-geom2d/content/geom2d/polygons2d/projPointOnPolygon.m
SDP solver for limit surface fitting:
http://cvxr.com/cvx/

Key references:

Zhou J, Paolini R, Bagnell JA and Mason MT (2016) A convex polynomial force-motion model for planar sliding: identification and application. In: 2016 IEEE International Conference on Robotics and Automation (ICRA). pp. 372–377.

Jiaji Zhou, J Andrew Bagnell, and Matthew T Mason. A fast stochastic contact model for planar pushing and grasping: Theory and experimental validation. In Robotics: Science and systems XIII, 2017.

Jiaji Zhou, R. Paolini, A. Johnson, J. A. Bagnell, and M. T. Mason. A probabilistic planning framework for planar grasping under uncertainty. In Intelligent Robots and Systems (IROS), 2017 IEEE/RSJ International Conference on, 2017.


