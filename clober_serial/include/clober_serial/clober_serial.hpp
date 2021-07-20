#ifndef __CLOBER_SERIAL_H__
#define __CLOBER_SERIAL_H__

#include <chrono>
#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <thread>
#include <memory>

#include <CppLinuxSerial/SerialPort.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>

// #include <tf2_ros/transform_broadcaster.h>




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

        mn::CppLinuxSerial::SerialPort serial_;

        // switch //
        bool cmd_vel_timeout_switch_;
        bool publish_tf_;

        // publisher //
        ros::Publisher odom_pub_;
        // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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
        

        ros::Time timestamp_;

        bool trigger_;

        shared_ptr<thread> readThread_;
        shared_ptr<thread> publishThread_;
        

};

#endif //__CLOBER_SERIAL_H__