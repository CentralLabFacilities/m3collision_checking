#include <pluginlib/class_loader.h>
#include <ros/ros.h>
// MoveIt!
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std;


int main(int argc, char** argv)
{
    ros::Time::init();
    ros::init(argc, argv, "m3collision_checking_node", ros::init_options::AnonymousName);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh("~");


    robot_model_loader::RobotModelLoader robot_model_loader("/robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    //robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    planning_scene::PlanningScene planning_scene(kinematic_model);


    planning_scene_monitor::PlanningSceneMonitor psm("/robot_description");
    psm.startStateMonitor("/joint_states");

    // Learn the allowed collision matrix (assusmes the robot is NOT in self collision state)
    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();


    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;
    collision_request.max_contacts = 1000;

    while(false == (psm.getStateMonitor()->waitForCurrentState(1.0))){
        usleep(100000);
    }
    robot_state::RobotStatePtr current_state = psm.getStateMonitor()->getCurrentState();
    robot_state::RobotState planning_state = planning_scene.getCurrentStateNonConst();
    double* pos=current_state->getVariablePositions();
    planning_state.setVariablePositions(pos);
    planning_scene.checkCollision(collision_request, collision_result,*current_state,acm);
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for(it = collision_result.contacts.begin();
        it != collision_result.contacts.end();
        ++it)
    {
        acm.setEntry(it->first.first, it->first.second, true);
        ROS_INFO("Contact between: %s and %s now allowed.",
                 it->first.first.c_str(),
                 it->first.second.c_str());
    }

    while(ros::ok()){
        if(psm.getStateMonitor()->waitForCurrentState(1.0)){
            collision_result.clear();

            current_state = psm.getStateMonitor()->getCurrentState();
            planning_state = planning_scene.getCurrentStateNonConst();
            pos=current_state->getVariablePositions();

            planning_state.setVariablePositions(pos);


            planning_scene.checkCollision(collision_request, collision_result,*current_state,acm);

            ROS_INFO_STREAM("Test 1: Current state is "
                            << (collision_result.collision ? "in" : "not in")
                            << " self collision");

            for(it = collision_result.contacts.begin();
                it != collision_result.contacts.end();
                ++it)
            {
                ROS_INFO("Contact between: %s and %s",
                         it->first.first.c_str(),
                         it->first.second.c_str());
            }
        }
    }
    ros::shutdown();
    return 0;
}
