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
#include "aubo_interface_msgs/msg/arm_joints.hpp"

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

#define SERVER_HOST "192.168.0.105"
#define SERVER_PORT 8899

/* AUBO ROBOT CONTROLLER */

class AuboController
{
    ServiceInterface robotService;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

public:
    void initializeRobot()
    {

        /** Interface call: login ***/
        ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
        if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cout << "login successful." << std::endl;
        }
        else
        {
            std::cerr << "login failed." << std::endl;
        }

        std::cout << "Robot arm initialization....." << std::endl;

        /** If the real robot arm is connected, the arm needs to be initialized.**/
        aubo_robot_namespace::ROBOT_SERVICE_STATE result;

        // Tool dynamics parameter
        aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
        memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

        ret = robotService.rootServiceRobotStartup(toolDynamicsParam /**Tool dynamics parameter**/,
                                                   6 /*Collision level*/,
                                                   true /*Whether to allow reading poses defaults to true*/,
                                                   true,    /*Leave the default to true */
                                                   1000,    /*Leave the default to 1000 */
                                                   result); /*Robot arm initialization*/
        if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cout << "Robot arm initialization succeeded." << std::endl;
        }
        else
        {
            std::cerr << "Robot arm initialization failed." << std::endl;
        }

        /** Business block **/
        /** Interface call: Initialize motion properties ***/
        robotService.robotServiceInitGlobalMoveProfile();

        /** Interface call: Set the maximum acceleration of the articulated motion ***/
        aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
        jointMaxAcc.jointPara[0] = 100.0 / 180.0 * M_PI;
        jointMaxAcc.jointPara[1] = 100.0 / 180.0 * M_PI;
        jointMaxAcc.jointPara[2] = 100.0 / 180.0 * M_PI;
        jointMaxAcc.jointPara[3] = 100.0 / 180.0 * M_PI;
        jointMaxAcc.jointPara[4] = 100.0 / 180.0 * M_PI;
        jointMaxAcc.jointPara[5] = 100.0 / 180.0 * M_PI; // The interface requires the unit to be radians
        robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

        /** Interface call: set the maximum speed of articulated motion ***/
        aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
        jointMaxVelc.jointPara[0] = 100.0 / 180.0 * M_PI;
        jointMaxVelc.jointPara[1] = 100.0 / 180.0 * M_PI;
        jointMaxVelc.jointPara[2] = 100.0 / 180.0 * M_PI;
        jointMaxVelc.jointPara[3] = 100.0 / 180.0 * M_PI;
        jointMaxVelc.jointPara[4] = 100.0 / 180.0 * M_PI;
        jointMaxVelc.jointPara[5] = 100.0 / 180.0 * M_PI; // The interface requires the unit to be radians
        robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);
    }

    void moveToZero()
    {
        /** Robot arm movement to zero posture **/
        double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
        Util::initJointAngleArray(jointAngle, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        std::cout << "Calling the motion function" << std::endl;
        ret = robotService.robotServiceJointMove(jointAngle, true);
        if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr << "Movement to zero posture failure.　ret:" << ret << std::endl;
        }
    }

    void setJointAngles(double joint0, double joint1, double joint2, double joint3, double joint4, double joint5)
    {
        /** Waypoint 1 movement **/
        double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
        Util::initJointAngleArray(jointAngle, joint0, joint1, joint2, joint3, joint4, joint5);
        robotService.robotServiceJointMove(jointAngle, false);
        if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr << "Movement to waypoint 1 failed.　ret:" << ret << std::endl;
        }
    }

    // Max vel is 180deg/s, max acc is 180deg/s^2

    void setJointMaxAcc(double joint0, double joint1, double joint2, double joint3, double joint4, double joint5)
    {
        /** Interface call: Set the maximum acceleration of the articulated motion ***/
        aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
        jointMaxAcc.jointPara[0] = joint0 / 180.0 * M_PI;
        jointMaxAcc.jointPara[1] = joint1 / 180.0 * M_PI;
        jointMaxAcc.jointPara[2] = joint2 / 180.0 * M_PI;
        jointMaxAcc.jointPara[3] = joint3 / 180.0 * M_PI;
        jointMaxAcc.jointPara[4] = joint4 / 180.0 * M_PI;
        jointMaxAcc.jointPara[5] = joint5 / 180.0 * M_PI; // The interface requires the unit to be radians
        robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
    }

    void setJointMaxVel(double joint0, double joint1, double joint2, double joint3, double joint4, double joint5)
    {
        /** Interface call: Set the maximum acceleration of the articulated motion ***/
        aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
        jointMaxVelc.jointPara[0] = joint0 / 180.0 * M_PI;
        jointMaxVelc.jointPara[1] = joint1 / 180.0 * M_PI;
        jointMaxVelc.jointPara[2] = joint2 / 180.0 * M_PI;
        jointMaxVelc.jointPara[3] = joint3 / 180.0 * M_PI;
        jointMaxVelc.jointPara[4] = joint4 / 180.0 * M_PI;
        jointMaxVelc.jointPara[5] = joint5 / 180.0 * M_PI; // The interface requires the unit to be radians
        robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);
    }

    double getJointAngles(int jointNum)
    {
        aubo_robot_namespace::JointStatus jointStatus[6];
        ret = robotService.robotServiceGetRobotJointStatus(jointStatus, 6);
        if (ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr << "Get Joint Angle Error　ret:" << ret << std::endl;
        }
        return jointStatus[jointNum].jointPosJ * 180.0 / M_PI;
    }

    void shutdownAndLogout()
    {
        /** Robotic arm shutdown **/
        robotService.robotServiceRobotShutdown();

        /** Interface call: logout　**/
        robotService.robotServiceLogout();
    }
};

/* ROS Publisher Node: Used to publish arm state info */

class MinimalPublisher : public rclcpp::Node
{

private:
    AuboController *controller;
    void publish_message()
    {
        auto message = aubo_interface_msgs::msg::ArmJoints();

        message.joint0 = controller->getJointAngles(0);
        message.joint1 = controller->getJointAngles(1);
        message.joint2 = controller->getJointAngles(2);
        message.joint3 = controller->getJointAngles(3);
        message.joint4 = controller->getJointAngles(4);
        message.joint5 = controller->getJointAngles(5);

        publisher_->publish(message);
    }

public:
    MinimalPublisher(AuboController *_controller)
        : Node("minimal_publisher"), count_(0)
    {
        controller = _controller;
        publisher_ = this->create_publisher<aubo_interface_msgs::msg::ArmJoints>("get_arm_joint_angles", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MinimalPublisher::publish_message, this));
    }

    rclcpp::Publisher<aubo_interface_msgs::msg::ArmJoints>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

/* ROS Subscriber Node: Used to receive arm state info */

class MinimalSubscriber : public rclcpp::Node
{

private:
    AuboController *controller;
    std::string topic;
    int type;
    void subscriber_callback(const aubo_interface_msgs::msg::ArmJoints::SharedPtr msg) const
    {
        switch (type)
        {
        case 0:
            controller->setJointAngles(msg->joint0 / 180.0 * M_PI, msg->joint1 / 180.0 * M_PI, msg->joint2 / 180.0 * M_PI, msg->joint3 / 180.0 * M_PI, msg->joint4 / 180.0 * M_PI, msg->joint5 / 180.0 * M_PI);
            break;
        case 1:
            controller->setJointMaxAcc(msg->joint0, msg->joint1, msg->joint2, msg->joint3, msg->joint4, msg->joint5);
            break;
        case 2:
            controller->setJointMaxVel(msg->joint0, msg->joint1, msg->joint2, msg->joint3, msg->joint4, msg->joint5);
            break;
        }
    }

public:
    MinimalSubscriber(AuboController *_controller, std::string _topic, int _type)
        : Node("minimal_subscriber")
    {
        controller = _controller;
        topic = _topic;
        type = _type;
        subscription_ = this->create_subscription<aubo_interface_msgs::msg::ArmJoints>(
            topic,
            10,
            std::bind(&MinimalSubscriber::subscriber_callback, this, std::placeholders::_1));
        std::cout << "ROS Subscriber now listening" << std::endl;
    }

    rclcpp::Subscription<aubo_interface_msgs::msg::ArmJoints>::SharedPtr subscription_;
};

/* MAIN */

int main(int argc, char *argv[])
{

    AuboController aubo_i5;

    aubo_i5.initializeRobot();

    rclcpp::init(argc, argv);

    // Create instances of the publisher and subscriber
    auto publisher = std::make_shared<MinimalPublisher>(&aubo_i5);
    auto angle_subscriber = std::make_shared<MinimalSubscriber>(&aubo_i5, "set_arm_joint_angles", 0);
    auto acc_subscriber = std::make_shared<MinimalSubscriber>(&aubo_i5, "set_arm_joint_acc", 1);
    auto vel_subscriber = std::make_shared<MinimalSubscriber>(&aubo_i5, "set_arm_joint_vel", 2);

    // Spin each node in a separate thread
    auto publisher_thread = std::make_shared<std::thread>([&]()
                                                          { rclcpp::spin(publisher); });

    auto angle_subscriber_thread = std::make_shared<std::thread>([&]()
                                                                 { rclcpp::spin(angle_subscriber); });

    auto acc_subscriber_thread = std::make_shared<std::thread>([&]()
                                                               { rclcpp::spin(acc_subscriber); });

    auto vel_subscriber_thread = std::make_shared<std::thread>([&]()
                                                               { rclcpp::spin(vel_subscriber); });

    // Wait for both threads to finish
    publisher_thread->join();
    angle_subscriber_thread->join();
    acc_subscriber_thread->join();
    vel_subscriber_thread->join();

    aubo_i5.shutdownAndLogout();

    return 0;
}
