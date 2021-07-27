#ifndef __CLOBER_SERIAL_H__
#define __CLOBER_SERIAL_H__

#include <chrono>
#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <thread>
#include <memory>
#include <serial/serial.h>
#include "clober_utils.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>


using namespace std::chrono_literals;
using namespace std;
using duration = std::chrono::nanoseconds;


const string eol("\r");
const size_t max_line_length(128);

struct Encoder{
    int ppr;
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

struct MotorCommand{
    float linearVel;   
    float angularVel;  
};


class CloberSerial{
    public:
        CloberSerial();
        ~CloberSerial();

        void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg);
        void updatePose();
        void updatePose(float dL, float dR);

        void on_motor_move(MotorCommand cmd);

        void SetValues();
        pair<float,float> toWheelSpeed(float v, float w);
        float toVW(float l_speed, float r_speed);

        float limitMaxSpeed(float speed);

        void read_serial(int ms);
        void parse();

        void publishOdom();
        void publish_loop(int hz);

        void sendRPM(pair<int,int> channel, pair<float,float> rpm);
        void sendStop(pair<int,int> channel);

    private:    
        ros::Time timestamp_;

        std::shared_ptr<serial::Serial> serial_;
        std::string port_;
        int32_t baudrate_;
        int timeout_;
        float control_frequency_;

        // switch //
        bool cmd_vel_timeout_switch_;
        bool publish_tf_;

        // publisher //
        ros::Publisher odom_pub_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;

        // Subscriber //
        ros::Subscriber cmd_vel_sub_;

        MotorCommand motor_cmd_;

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

        bool trigger_;

        shared_ptr<thread> readThread_;
        shared_ptr<thread> publishThread_;

        CloberUtils utils_;

        int odom_mode_;
};

#endif //__CLOBER_SERIAL_H__