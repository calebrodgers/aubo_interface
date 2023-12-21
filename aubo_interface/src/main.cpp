#include <iostream>

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include "util.h"

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "aubo_interface_msgs/msg/joint_angles.hpp"

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"


#define SERVER_HOST "192.168.0.105"
#define SERVER_PORT 8899

using std::placeholders::_1;

class AuboController
{
    ServiceInterface robotService;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;
    public:
    void initializeRobot() {
        

    /** Interface call: login ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"login successful."<<std::endl;
    }
    else
    {
        std::cerr<<"login failed."<<std::endl;
    }

     std::cout<<"Robot arm initialization....."<<std::endl;


     /** If the real robot arm is connected, the arm needs to be initialized.**/
     aubo_robot_namespace::ROBOT_SERVICE_STATE result;

     //Tool dynamics parameter
     aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
     memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

     ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**Tool dynamics parameter**/,
                                                6        /*Collision level*/,
                                                true     /*Whether to allow reading poses defaults to true*/,
                                                true,    /*Leave the default to true */
                                                1000,    /*Leave the default to 1000 */
                                                result); /*Robot arm initialization*/
     if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
     {
         std::cout<<"Robot arm initialization succeeded."<<std::endl;
     }
     else
     {
         std::cerr<<"Robot arm initialization failed."<<std::endl;
     }

    /** Business block **/
    /** Interface call: Initialize motion properties ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** Interface call: Set the maximum acceleration of the articulated motion ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[1] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[2] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[3] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[4] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[5] = 50.0/180.0*M_PI;   //The interface requires the unit to be radians
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    /** Interface call: set the maximum speed of articulated motion ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[1] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[2] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[3] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[4] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[5] = 50.0/180.0*M_PI;   //The interface requires the unit to be radians
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    }

    void moveToZero() {
        /** Robot arm movement to zero posture **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    Util::initJointAngleArray(jointAngle, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    std::cout<<"Calling the motion function"<<std::endl;
    ret = robotService.robotServiceJointMove(jointAngle, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"Movement to zero posture failure.　ret:"<<ret<<std::endl;
    }
    }

    void setJointAngles(double joint0,double joint1,double joint2,double joint3,double joint4,double joint5) {
                 /** Waypoint 1 movement **/
                 double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
        Util::initJointAngleArray(jointAngle, joint0, joint1, joint2, joint3, joint4, joint5);
        robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"Movement to waypoint 1 failed.　ret:"<<ret<<std::endl;
            // break;
        }
    }

    void shutdownAndLogout() {
        /** Robotic arm shutdown **/
        robotService.robotServiceRobotShutdown();

        /** Interface call: logout　**/
        robotService.robotServiceLogout();
    }
};

class MinimalSubscriber : public rclcpp::Node
{
    private:
    AuboController* controller;
        void topic_callback(const aubo_interface_msgs::msg::JointAngles::SharedPtr msg) const
        {
            // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            controller->setJointAngles(msg->joint0/180.0*M_PI,  msg->joint1/180.0*M_PI,  msg->joint2/180.0*M_PI, msg->joint3/180.0*M_PI, msg->joint4/180.0*M_PI, msg->joint5/180.0*M_PI);
            
        }
        rclcpp::Subscription<aubo_interface_msgs::msg::JointAngles>::SharedPtr subscription_;
    public:
        MinimalSubscriber(AuboController* _controller)
        : Node("minimal_subscriber")
        {
            controller = _controller;
            subscription_ = this->create_subscription<aubo_interface_msgs::msg::JointAngles>(
                "arm_joint_angles",10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
            std::cout<<"ROS Subscriber now listening"<<std::endl;
        }
};



int main(int argc, char * argv[]) {

    AuboController aubo_i5;

    aubo_i5.initializeRobot();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>(&aubo_i5));
    rclcpp::shutdown();


    aubo_i5.shutdownAndLogout();

return 0;
}
