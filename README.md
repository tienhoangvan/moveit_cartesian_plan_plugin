RViz and RQT User Interface:
There are two types of interactive markers:
  - The red arrow acts as a pointer which the user can move around the RViz enviroment. Fruthermore by clicking on the arrow another blue arrow is added to the RViz enviroment. This arrow acts as way-point for the Cartesian Planner.
  - The blue arrow is the  way point for the cartesian trajectory planning. The orientation of the arrow can be changed by holding the CTRL key and moving it with the mouse.
  - Each arrow has a menu where the user can either delete the selected arrow or it can change its position and orientation by using the 6DOF marker control.
  - When the way-point is out of the IK solution for the Robot the arrow changes its color from blue to yellow.
  - The RQT UI communicates simultaniously with the RViz enviroment and the User can change the state of a marker either through RViz or the RQT UI 
  - TreeView displays all the added waypoints. The user can manipulate them directly in the TreeView and see their position and orientation of each waypoint.
  - The user can add new point or delete it through the RQT UI.
  - New tool component has been added for adding Arrows by using a mouse click
