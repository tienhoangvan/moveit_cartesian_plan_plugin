#include <boost/function.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <math.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_cartesian_plan_plugin/generate_cartesian_path.hpp>


#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>


GenerateCartesianPath::GenerateCartesianPath(QObject *parent)
{
    //to add initializations
    init();

}
GenerateCartesianPath::~GenerateCartesianPath()
{
  /*! The destructor resets the moveit_group_ and the kinematic_state of the robot.
  */
  moveit_group_.reset();
  kinematic_state_.reset();
  robot_model_loader.reset();
  kmodel_.reset();
}

void GenerateCartesianPath::init()
{
  /*! Initialize the MoveIt parameters:
        - MoveIt group
        - Kinematic State is the current kinematic congiguration of the Robot
        - Robot model which handles getting the Robot Model
        - Joint Model group which are necessary for checking if Way-Point is outside the IK Solution
        .
  */
  selected_plan_group = 0;
  robot_model_loader = RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
  kmodel_ = robot_model_loader->getModel();

  end_eff_joint_groups = kmodel_->getEndEffectors();


  ROS_INFO_STREAM("size of the end effectors is: "<<end_eff_joint_groups.size());

  if (end_eff_joint_groups.empty())
  {
    std::vector< std::string > group_names_tmp_;
    const moveit::core::JointModelGroup *  end_eff_joint_groups_tmp_;
    group_names_tmp_ = kmodel_->getJointModelGroupNames();

    for(int i=0;i<group_names_tmp_.size();i++)
    {

    end_eff_joint_groups_tmp_ = kmodel_->getJointModelGroup(group_names_tmp_.at(i));
    if(end_eff_joint_groups_tmp_->isChain())
    {
      group_names.push_back(group_names_tmp_.at(i));
    }
    else
    {
      ROS_WARN_STREAM("The group:" << end_eff_joint_groups_tmp_->getName() <<" is not a Chain. Depreciate it!!");
    }
    }
  }
  else
  {
  for(int i=0;i<end_eff_joint_groups.size();i++)
  {
    if(end_eff_joint_groups.at(i)->isChain())
    {
    const std::string& parent_group_name = end_eff_joint_groups.at(i)->getName();
    group_names.push_back(parent_group_name);

    ROS_INFO_STREAM("Group name:"<< group_names.at(i));
    }
    else
    {
        ROS_INFO_STREAM("This group is not a chain. Find the parent of the group");
        const std::pair< std::string, std::string > & parent_group_name = end_eff_joint_groups.at(i)->getEndEffectorParentGroup();
        group_names.push_back(parent_group_name.first);
    }
  }
}

  ROS_INFO_STREAM("Group name:"<< group_names[selected_plan_group]);

  moveit_group_ = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(group_names[selected_plan_group]));
  kinematic_state_ = moveit::core::RobotStatePtr(new robot_state::RobotState(kmodel_));
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = kmodel_->getJointModelGroup(group_names[selected_plan_group]);
}

void GenerateCartesianPath::setCartParams(double plan_time_,double cart_step_size_, double cart_jump_thresh_, bool moveit_replan_,bool avoid_collisions_)
{
  /*! Set the necessary parameters for the MoveIt and the Cartesian Path Planning.
      These parameters correspond to the ones that the user has entered or the default ones before the execution of the Cartesian Path Planner.
  */
  ROS_INFO_STREAM("MoveIt and Cartesian Path parameters from UI:\n MoveIt Plan Time:"<<plan_time_
                  <<"\n Cartesian Path Step Size:"<<cart_step_size_
                  <<"\n Jump Threshold:"<<cart_jump_thresh_
                  <<"\n Replanning:"<<moveit_replan_
                  <<"\n Avoid Collisions:"<<avoid_collisions_);

  PLAN_TIME_        = plan_time_;
  MOVEIT_REPLAN_    = moveit_replan_;
  CART_STEP_SIZE_   = cart_step_size_;
  CART_JUMP_THRESH_ = cart_jump_thresh_;
  AVOID_COLLISIONS_ = avoid_collisions_;
}

void GenerateCartesianPath::moveToPose(std::vector<geometry_msgs::Pose> waypoints)
{
    /*!

    */
    Q_EMIT cartesianPathExecuteStarted();

    moveit_group_->setPlanningTime(PLAN_TIME_);
    moveit_group_->allowReplanning (MOVEIT_REPLAN_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    moveit_msgs::RobotTrajectory trajectory_;
    double fraction = moveit_group_->computeCartesianPath(waypoints,CART_STEP_SIZE_,CART_JUMP_THRESH_,trajectory_,AVOID_COLLISIONS_);
    robot_trajectory::RobotTrajectory rt(kmodel_, group_names[selected_plan_group]);

    rt.setRobotTrajectoryMsg(*kinematic_state_, trajectory_);

    ROS_INFO_STREAM("Pose reference frame: " << moveit_group_->getPoseReferenceFrame ());

    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");


    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory_);
    // Finally plan and execute the trajectory
    plan.trajectory_ = trajectory_;
    ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",fraction * 100.0);
    Q_EMIT cartesianPathCompleted(fraction);

    moveit_group_->execute(plan);

    kinematic_state_ = moveit_group_->getCurrentState();

    Q_EMIT cartesianPathExecuteFinished();

}

void GenerateCartesianPath::moveToJoint(std::vector<double> joints)     // TIEN VIET THEM
{
    /*!
      Move the arm to a joint-space goal
      Let's set a joint space goal and move towards it.  This will replace the pose target we set above.
      To start, we'll create an pointer that references the current robot's state.
      RobotState is the object that contains all the current position/velocity/acceleration data.
    */

      Q_EMIT JointSpaceExecuteStarted();

      kinematic_state_ = moveit_group_->getCurrentState();


     // Next get the current set of joint values for the group.
      std::vector<double> joint_group_positions;

      kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_group_positions);

      joint_group_positions = joints;  // radians
      
      moveit_group_->setJointValueTarget(joint_group_positions);

      ROS_INFO("Start moving joints");

      moveit_group_->move();

      kinematic_state_ = moveit_group_->getCurrentState();

      // Calling ``stop()`` ensures that there is no residual movement
      moveit_group_->stop();

      ROS_INFO("Moving joints has done");

      Q_EMIT JointSpaceExecuteFinished();

}

void GenerateCartesianPath::getCurrentJoints()     // TIEN VIET THEM
{
    /*!
      Get current Joint States
    */
      ROS_INFO("Getting current joint positions...");

      Q_EMIT JointSpaceExecuteStarted();

      kinematic_state_ = moveit_group_->getCurrentState();

     // Next get the current set of joint values for the group.
      //std::vector<double> joint_group_positions;

      //kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_group_positions);

      std::vector<double> current_joints;

      kinematic_state_->copyJointGroupPositions(joint_model_group_, current_joints);
      
      ROS_INFO("Got current joint positions");

      double q1, q2, q3, q4, q5, q6;
      q1 = current_joints[0];
      q2 = current_joints[1];
      q3 = current_joints[2];
      q4 = current_joints[3];
      q5 = current_joints[4];
      q6 = current_joints[5];

      /*
      ROS_INFO("Joint %s: %f", "joint 1", q1);
      ROS_INFO("Joint %s: %f", "joint 2", q2);
      ROS_INFO("Joint %s: %f", "joint 3", q3);
      ROS_INFO("Joint %s: %f", "joint 4", q4);
      ROS_INFO("Joint %s: %f", "joint 5", q5);
      ROS_INFO("Joint %s: %f", "joint 6", q6);
      */

      //Q_EMIT updateJoint(current_joints);
      Q_EMIT updateJoint(q1, q2, q3, q4, q5, q6);
      Q_EMIT JointSpaceExecuteFinished();

}

void GenerateCartesianPath::cartesianPathHandler(std::vector<geometry_msgs::Pose> waypoints)
{
  /*! Since the execution of the Cartesian path is time consuming and can lead to locking up of the Plugin and the RViz enviroment the function for executing the Cartesian Path Plan has been placed in a separtate thread.
      This prevents the RViz and the Plugin to lock.
  */
  ROS_INFO("Starting concurrent process for Cartesian Path");
  QFuture<void> future = QtConcurrent::run(this, &GenerateCartesianPath::moveToPose, waypoints);
}


void GenerateCartesianPath::JointSpaceHandler(std::vector<double> joints)
{
  /*! Since the execution of the Joint Space is time consuming and can lead to locking up of the Plugin and the RViz enviroment the function for executing the Cartesian Path Plan has been placed in a separtate thread.
      This prevents the RViz and the Plugin to lock.
  */
  ROS_INFO("Starting moving to joint goals");
  QFuture<void> future = QtConcurrent::run(this, &GenerateCartesianPath::moveToJoint, joints);
}

void GenerateCartesianPath::JointSpaceUpdate()
{
  /*! Since the execution of the Joint Space is time consuming and can lead to locking up of the Plugin and the RViz enviroment the function for executing the Cartesian Path Plan has been placed in a separtate thread.
      This prevents the RViz and the Plugin to lock.
  */
  ROS_INFO("Starting getting current joint states");
  QFuture<void> future = QtConcurrent::run(this, &GenerateCartesianPath::getCurrentJoints);
}

void GenerateCartesianPath::checkWayPointValidity(const geometry_msgs::Pose& waypoint,const int point_number)
{
      /*! This function is called every time the user updates the pose of the Way-Point and checks if the Way-Point is within the valid IK solution for the Robot.
          In the case when a point is outside the valid IK solution this function send a signal to the RViz enviroment to update the color of the Way-Point.
      */
       bool found_ik = kinematic_state_->setFromIK(joint_model_group_, waypoint, 3, 0.006);

         if(found_ik)
        {

           Q_EMIT wayPointOutOfIK(point_number,0);
        }
        else
        {
          Q_EMIT wayPointOutOfIK(point_number,1);
        }
}

void GenerateCartesianPath::initRvizDone()
{
  /*! Once the initialization of the RViz is has finished, this function sends the pose of the robot end-effector and the name of the base frame to the RViz enviroment.
      The RViz enviroment sets the User Interactive Marker pose and Add New Way-Point RQT Layout default values based on the end-effector starting position.
      The transformation frame of the InteractiveMarker is set based on the robot PoseReferenceFrame.
  */
  ROS_INFO("RViz is done now we need to emit the signal");

  if(moveit_group_->getEndEffectorLink().empty())
  {
    ROS_INFO("End effector link is empty");
    const std::vector< std::string > &  joint_names = joint_model_group_->getLinkModelNames();
    for(int i=0;i<joint_names.size();i++)
    {
      ROS_INFO_STREAM("Link " << i << " name: "<< joint_names.at(i));
    }
    const Eigen::Affine3d &end_effector_state = kinematic_state_->getGlobalLinkTransform(joint_names.at(0));
    //tf::Transform end_effector;
    tf::transformEigenToTF(end_effector_state, end_effector);
    Q_EMIT getRobotModelFrame_signal(moveit_group_->getPoseReferenceFrame(),end_effector);
  }
  else
  {

    ROS_INFO("End effector link is not empty");
    const Eigen::Affine3d &end_effector_state = kinematic_state_->getGlobalLinkTransform(moveit_group_->getEndEffectorLink());
    //tf::Transform end_effector;
    tf::transformEigenToTF(end_effector_state, end_effector);

    Q_EMIT getRobotModelFrame_signal(moveit_group_->getPoseReferenceFrame(),end_effector);
  }

    Q_EMIT sendCartPlanGroup(group_names);


}
void GenerateCartesianPath::moveToHome()
{

  geometry_msgs::Pose home_pose;
  tf::poseTFToMsg(end_effector,home_pose);

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(home_pose);

  cartesianPathHandler(waypoints);

}

void GenerateCartesianPath::getSelectedGroupIndex(int index)
{
  selected_plan_group = index;

  ROS_INFO_STREAM("selected name is:"<<group_names[selected_plan_group]);
  moveit_group_.reset();
  kinematic_state_.reset();
  moveit_group_ = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(group_names[selected_plan_group]));

  kinematic_state_ = moveit::core::RobotStatePtr(new robot_state::RobotState(kmodel_));
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = kmodel_->getJointModelGroup(group_names[selected_plan_group]);

  ROS_INFO("End effector link is not empty");
  const Eigen::Affine3d &end_effector_state = kinematic_state_->getGlobalLinkTransform(moveit_group_->getEndEffectorLink());
  tf::transformEigenToTF(end_effector_state, end_effector);

  Q_EMIT getRobotModelFrame_signal(moveit_group_->getPoseReferenceFrame(),end_effector);

}
