#ifndef __CLOBER_SERIAL_H__
#define __CLOBER_SERIAL_H__

#include <chrono>
#include <string>
#include <bitset>
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
#include <clober_msgs/Feedback.h>

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
        void updatePose(double dL, double dR);

        void on_motor_move(MotorCommand cmd);

        void faultFlags(const uint16_t flags);

        void SetValues();
        pair<float,float> toWheelSpeed(float v, float w);
        void toVW(float l_speed, float r_speed);

        float limitMaxSpeed(float speed);

        void read_serial(int ms);
        void parse();

        void publishOdom();
        void publishFeedback();
        void publish_loop(int hz);

        void sendRPM(pair<int,int> channel, pair<float,float> rpm);
        void sendStop(pair<int,int> channel);

        void sendHeardBeat();
        void restartScript();


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
        ros::Publisher feedback_pub_;
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
        
        double posX_;
        double posY_;
        float heading_;

        bool trigger_;

        shared_ptr<thread> readThread_;
        shared_ptr<thread> publishThread_;

        CloberUtils utils_;

        int odom_mode_;
};

#endif //__CLOBER_SERIAL_H__