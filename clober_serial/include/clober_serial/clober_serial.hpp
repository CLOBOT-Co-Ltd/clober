#ifndef __CLOBER_SERIAL_H__
#define __CLOBER_SERIAL_H__

#include <chrono>
#include <string>
#include <bitset>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <thread>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <clober_msgs/msg/feedback.hpp>

#include "clober_utils.hpp"


using namespace std::chrono_literals;
using namespace std;
using duration = std::chrono::nanoseconds;

#define PI 3.141592

const string eol("\r");
const size_t max_line_length(128);


struct Encoder{
    int ppr;

};


struct ControllerState{
    bool emergency_stop;
    float temperature;
    float battery_voltage;
    float charging_voltage;
    float current_12v;
    float current_24v;
    vector<string> fault_flags;
};

struct MotorState{
    float speed;
    float rpm;
    float position_rad;
    float position_meter_prev;
    float position_meter_curr;
    float current;
};

struct VehicleConfig{
    float WIDTH;
    float WheelRadius;
    float MAX_SPEED;
    float MAX_RPM;
    Encoder encoder;
    MotorState left_motor;
    MotorState right_motor;
    ControllerState controller_state;
};


class CloberSerial : public rclcpp::Node{
    public:
        CloberSerial();
        ~CloberSerial();

        void cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr msg);
        void updatePose();
        void updatePose(float dL, float dR);

        void on_motor_move(geometry_msgs::msg::Twist::SharedPtr msg);

        void faultFlags(const uint16_t flags);

        void SetValues();
        pair<float,float> toWheelSpeed(float v, float w);
        float toVW(float l_speed, float r_speed);

        void read_serial(int ms);
        void parse();

        void publishOdom();
        void publishFeedback();
        void publish_loop(int ms);

        float limitMaxSpeed(float speed);

        void sendRPM(pair<int,int> channel, pair<float,float> rpm);
        void sendStop(pair<int,int> channel);


    private:
        rclcpp::Node::SharedPtr node_handle_;

        // switch //
        bool cmd_vel_timeout_switch_;
        bool publish_tf_;
        float control_frequency_;

        // timer //
        rclcpp::TimerBase::SharedPtr odom_read_timer_;
        rclcpp::TimerBase::SharedPtr ser_read_timer_;

        // publisher //
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<clober_msgs::msg::Feedback>::SharedPtr feedback_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // Subscriber //
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

        std::shared_ptr<serial::Serial> serial_;
        std::string port_;
        int32_t baudrate_;

        std::shared_ptr<geometry_msgs::msg::Twist> motor_cmd_;

        double odom_freq_;
        double cmd_vel_timeout_;

        std::string odom_frame_parent_;
        std::string odom_frame_child_;

        VehicleConfig config_;

        float linearVel_;
        float angularVel_;
        
        float posX_;
        float posY_;
        float heading_;
        

        rclcpp::Time timestamp_;

        bool trigger_;

        shared_ptr<thread> readThread_;
        shared_ptr<thread> publishThread_;
        
        CloberUtils utils_;

        int odom_mode_;
};

#endif //__CLOBER_SERIAL_H__