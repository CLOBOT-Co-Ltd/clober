#ifndef __CLOBER_SERIAL_H__
#define __CLOBER_SERIAL_H__

#include <chrono>
#include <string>
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




using namespace std::chrono_literals;
using namespace std;
using duration = std::chrono::nanoseconds;

#define PI 3.141592

const string eol("\r");
const size_t max_line_length(128);


struct Encoder{
    int cpr;

};


struct MotorState{
    float speed;
    float rpm;
    float position_rad;
    float position_meter_prev;
    float position_meter_curr;
};


struct VehicleConfig{
    float WIDTH;
    float WheelRadius;
    float MAX_SPEED;
    float MAX_RPM;
    Encoder encoder;
    MotorState left_motor;
    MotorState right_motor;
};


class CloberSerial : public rclcpp::Node{
    public:
        CloberSerial();
        ~CloberSerial();

        void cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr msg);
        void updatePose();
        void updatePose(float dL, float dR);


        void on_motor_move(geometry_msgs::msg::Twist::SharedPtr msg);

        void SetValues();
        pair<float,float> toWheelSpeed(float v, float w);
        float toVW(float l_speed, float r_speed);

        float limitMaxSpeed(float speed);
        float toRPM(float w);
        float toVelocity(float rpm);
        float toRad(float enc);

        void writeVelocity(int channel, float rpm);
        void stopMotor(int channel);
        void read_serial(int ms);
        void parse();

        void publishOdom();
        void publish_loop(int ms);



    private:
        rclcpp::Node::SharedPtr node_handle_;

        // switch //
        bool cmd_vel_timeout_switch_;
        bool publish_tf_;

        // timer //
        rclcpp::TimerBase::SharedPtr odom_read_timer_;
        rclcpp::TimerBase::SharedPtr ser_read_timer_;

        // publisher //
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // Subscriber //
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

        std::shared_ptr<serial::Serial> serial_;
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
        

};

#endif //__CLOBER_SERIAL_H__