#include "clober_serial/clober_serial.hpp"


CloberSerial::CloberSerial():Node("clober_serial", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    std::string port = declare_parameter("port","/dev/ttyUSB0");
    int32_t baudrate = declare_parameter("baudrate", 115200);
    odom_freq_ = declare_parameter("odom_freq",50.0);
    odom_frame_parent_ = declare_parameter("odom_frame_parent","odom");
    odom_frame_child_ = declare_parameter("odom_frame_child","base_link");
    publish_tf_ = declare_parameter("publish_tf",true);
    cmd_vel_timeout_ = declare_parameter("cmd_vel_timeout",1.0);

    SetValues();

    RCLCPP_INFO(get_logger(), "Connecting to serial : '%s', with baudrate '%d'", port.c_str(), baudrate);
    try{
        serial_ = std::make_unique<serial::Serial>(port, baudrate);

        serial::Timeout to = serial::Timeout::simpleTimeout(0); 
        serial_->setTimeout(to);
        // serial_->setTimeout(0,500,0,500,0);

        // serial_->open();

    }catch(const std::exception &e){
        RCLCPP_ERROR(get_logger(),"Connection failed");
        RCLCPP_ERROR(get_logger(), e.what());
        throw;
    }


    if(serial_->isOpen()){
        RCLCPP_INFO(get_logger(), "Connected to serial : '%s', with baudrate '%d'", port.c_str(), baudrate);
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom",rclcpp::QoS(1));
    motor_cmd_ = std::make_unique<geometry_msgs::msg::Twist>();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",rclcpp::QoS(rclcpp::KeepLast(10)), [=](geometry_msgs::msg::Twist::SharedPtr msg){cmd_vel_callback(msg);});
    // cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",rclcpp::QoS(1), [=](geometry_msgs::msg::Twist::SharedPtr msg){cmd_vel_callback(msg);});

    // ser_read_timer_ = create_wall_timer(1s/odom_freq_, [=](){parse();});
    // ser_read_timer_ = create_wall_timer(1s/10, [=](){parse();});

    readThread_ = std::make_shared<thread>(bind(&CloberSerial::read_serial, this, 20));
    // publishThread_ = std::make_shared<thread>(bind(&CloberSerial::publish_loop, this, 20));
}

CloberSerial::~CloberSerial(){
    readThread_->detach();
    if( readThread_->joinable()){
        readThread_->join();
    }

}

void CloberSerial::SetValues(){
    config_.WIDTH = 0.45;
    config_.WheelRadius = 0.085;
    config_.encoder.cpr = 4096;
    config_.MAX_SPEED = 1.0;
    config_.MAX_RPM = 200;
    config_.left_motor.position_rad = 0.0;
    config_.right_motor.position_rad = 0.0;
    trigger_ = false;
    posX_ = 0.0;
    posY_ = 0.0;
    heading_ = 0.0;

}

void CloberSerial::read_serial(int ms){
    while(rclcpp::ok()){
        parse();
        // std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
}


void CloberSerial::parse(){
    // if( serial_->available()){
        string msg = serial_->readline(max_line_length, eol);
        if( msg.size() > 2 ){
            if( msg[0] == 'F'  && msg[2] == ':'){

                try{
                    size_t index = boost::lexical_cast<size_t>(msg[1]);

                    vector<string> feedbacks;
                    boost::split(feedbacks, msg, boost::algorithm::is_any_of(":"));

                    if( feedbacks.size() > 8 ){

                        config_.left_motor.rpm = boost::lexical_cast<float>(feedbacks[5]);
                        config_.left_motor.speed = toVelocity(boost::lexical_cast<float>(feedbacks[5]));
                        
                        config_.right_motor.rpm = boost::lexical_cast<float>(feedbacks[6]);
                        config_.right_motor.speed = toVelocity(boost::lexical_cast<float>(feedbacks[6]));           
                        
                        config_.left_motor.position_rad += toRad(boost::lexical_cast<float>(feedbacks[7]));
                        config_.left_motor.position_meter_curr = config_.left_motor.position_rad * config_.WheelRadius;

                        config_.right_motor.position_rad += toRad(boost::lexical_cast<float>(feedbacks[8]));
                        config_.right_motor.position_meter_curr = config_.right_motor.position_rad * config_.WheelRadius;

                        float dl = config_.left_motor.position_meter_curr - config_.left_motor.position_meter_prev;
                        float dr = config_.right_motor.position_meter_curr - config_.right_motor.position_meter_prev;
                        
                        float l_speed = config_.left_motor.speed * config_.WheelRadius;
                        float r_speed = config_.right_motor.speed * config_.WheelRadius;
                        
                        // updatePose(dl, dr);
                        // updatePose(l_speed, r_speed);

                        cout <<"enc l : "<< boost::lexical_cast<float>(feedbacks[7]) << endl;
                        cout <<"enc l position(rad) : "<< config_.left_motor.position_rad << endl;
                        cout <<"enc l position(m) : "<< config_.left_motor.position_meter_curr << endl;

                        cout <<"enc r : "<< boost::lexical_cast<float>(feedbacks[8]) << endl;
                        cout <<"enc r position(rad) : "<< config_.right_motor.position_rad << endl;
                        cout <<"enc r position(m) : "<< config_.right_motor.position_meter_curr << endl;

                        config_.left_motor.position_meter_prev = config_.left_motor.position_meter_curr;
                        config_.right_motor.position_meter_prev = config_.right_motor.position_meter_curr;

                        toVW(config_.left_motor.speed, config_.right_motor.speed);
                        updatePose();
                    }
                }
                catch(boost::bad_lexical_cast &e){
                }
            }
        }
    // }
}


void CloberSerial::cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr msg){
    motor_cmd_->linear.x = msg->linear.x;
    motor_cmd_->angular.z = msg->angular.z;
    cmd_vel_timeout_switch_ = false;

    cout <<"cmd_vel callback"<<endl;

    on_motor_move(motor_cmd_);
}


float CloberSerial::toVW(float l_speed, float r_speed){
    // rad/s -> m/s 
    l_speed *= config_.WheelRadius;
    r_speed *= config_.WheelRadius;

    linearVel_ = ( l_speed + r_speed ) / 2;
    angularVel_ = ( r_speed - l_speed ) / config_.WIDTH;

    // RCLCPP_INFO(get_logger()," linear vel : %f", linearVel_);
    // RCLCPP_INFO(get_logger()," angular vel : %f", angularVel_);
}


pair<float,float> CloberSerial::toWheelSpeed(float v, float w){
    // v, w -> wheel angular speed (rad/s)
    float r_speed = ( v + (config_.WIDTH * w/2) ) /config_.WheelRadius;
    float l_speed = ( v - (config_.WIDTH * w/2) ) /config_.WheelRadius;
    
    return make_pair(l_speed, r_speed);
}

float CloberSerial::limitMaxSpeed(float speed){
    float max = config_.MAX_SPEED / config_.WheelRadius;   // wheel angular max speed (rad/s)

    if( abs(speed) > max){
        if ( speed > 0 ){
            return max;
        }else{
            return -max;
        }
    }else{
        return speed;
    }
}

float CloberSerial::toRPM(float w){
    float rpm = (w*60.0) / (2*PI);
    return rpm;
}


float CloberSerial::toVelocity(float rpm){
    float w = (rpm * 2.0 * PI) / 60.0;
    return w;
}


float CloberSerial::toRad(float enc){
    float rad = (enc * 2.0 * PI) / config_.encoder.cpr; 
    return rad;
}


void CloberSerial::writeVelocity(int channel, float rpm){
    stringstream msg;

    msg << "!G " << channel+1 << " " <<  rpm << "\r";
    // cout <<"writeVelocity : "<<msg.str()<<endl;

    serial_->write(msg.str());
}


void CloberSerial::stopMotor(int channel){
    stringstream msg;
    msg << "!MS " << channel+1 << "\r";
    // cout <<"stopMotor : "<<msg.str()<<endl;

    serial_->write(msg.str());
}


void CloberSerial::updatePose(){
    auto now = get_clock()->now();

    if (!trigger_){
        timestamp_ = now;
        trigger_ = true;
        return;
    }

    double dT = now.seconds() - timestamp_.seconds();
    timestamp_ = now;

    float x = linearVel_ * dT * cos(heading_);
    float y = linearVel_ * dT * sin(heading_);
    float theta = angularVel_ * dT;

    posX_ += x;
    posY_ += y;
    heading_ += theta;

    cout <<"update pose x : "<<posX_ <<", y : "<<posY_<<endl;
}

void CloberSerial::updatePose(float dL, float dR){
    auto now = get_clock()->now();

    if (!trigger_){
        timestamp_ = now;
        trigger_ = true;
        return;
    }

    double dT = now.seconds() - timestamp_.seconds();
    timestamp_ = now;

    float x = posX_;
    float y = posY_;
    float theta = heading_;

    float R = 0.0;
    if ( (dR - dL) < 0.0001 ){
        R = 0.0;
    }else{
        R = (config_.WIDTH / 2.0) * ( (dL+dR)/(dR-dL) );
    }

    // float Wdt = (dR - dL) / config_.WIDTH;

    float W = (dR - dL) / config_.WIDTH;
    float Wdt = W*dT;



    float ICCx = x - (R*sin(theta));
    float ICCy = y + (R*cos(theta));

    posX_ = (cos(Wdt)*(x - ICCx)) - (sin(Wdt)*(y - ICCy)) + ICCx;
    posY_ = (sin(Wdt)*(x - ICCx)) + (cos(Wdt)*(y - ICCy)) + ICCy;
    heading_ = theta + Wdt;

    cout <<"update pose x : "<<posX_ <<", y : "<<posY_<<endl;
}


void CloberSerial::publish_loop(int ms){
    while(rclcpp::ok()){
        publishOdom();
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
}


void CloberSerial::publishOdom(){

    tf2::Quaternion q;
    q.setRPY(0.0,0.0,heading_);
    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    odom->header.frame_id = odom_frame_parent_;
    odom->child_frame_id = odom_frame_child_;
    odom->header.stamp = timestamp_;
    odom->pose.pose.position.x = posX_;
    odom->pose.pose.position.y = posY_;
    odom->pose.pose.orientation.x = q.x();
    odom->pose.pose.orientation.y = q.y();
    odom->pose.pose.orientation.z = q.z();
    odom->pose.pose.orientation.w = q.w();
    odom->pose.covariance.fill(0.0);
    odom->pose.covariance[0] = 1e-3;
    odom->pose.covariance[7] = 1e-3;
    odom->pose.covariance[14] = 1e6; 
    odom->pose.covariance[21] = 1e6; 
    odom->pose.covariance[28] = 1e6;
    odom->pose.covariance[35] = 1e-3;

    odom->twist.twist.linear.x = linearVel_;
    odom->twist.twist.angular.z = angularVel_;
    odom->twist.covariance.fill(0.0);
    odom->twist.covariance[0] = 1e-3;
    odom->twist.covariance[7] = 1e-3;
    odom->twist.covariance[14] = 1e6;
    odom->twist.covariance[21] = 1e6;
    odom->twist.covariance[28] = 1e6;
    odom->twist.covariance[35] = 1e3;

    if(publish_tf_){
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = timestamp_;
        odom_tf.header.frame_id = odom_frame_parent_;
        odom_tf.child_frame_id = odom_frame_child_;
        odom_tf.transform.translation.x = posX_;
        odom_tf.transform.translation.y = posY_;
        odom_tf.transform.translation.z = 0;
        odom_tf.transform.rotation = odom->pose.pose.orientation;
        tf_broadcaster_->sendTransform(odom_tf);       
    }

    odom_pub_->publish(std::move(odom));

    cout <<"odom x : "<<posX_<<", y : "<<posY_<<", heading : "<<heading_<<endl;


    // static int timeout_counter = 0;
    // if( !cmd_vel_timeout_switch_){
    //     timeout_counter = 0;
    //     cmd_vel_timeout_switch_ = true;
    // }else{
    //     if( timeout_counter > cmd_vel_timeout_*odom_freq_){
    //         motor_cmd_->linear.x = 0;
    //         motor_cmd_->angular.z = 0;
    //         timeout_counter = 0;
    //     }else{
    //         timeout_counter++;
    //     }
    // }

    // on_motor_move(motor_cmd_);
}



void CloberSerial::on_motor_move(geometry_msgs::msg::Twist::SharedPtr msg){

    pair<float, float> wheel_speed;
    wheel_speed = toWheelSpeed(msg->linear.x, msg->angular.z);
    wheel_speed.first = limitMaxSpeed(wheel_speed.first);
    wheel_speed.second = limitMaxSpeed(wheel_speed.second);

    pair<float, float> wheel_rpm;
    wheel_rpm.first = toRPM(wheel_speed.first) * 1000 / config_.MAX_RPM;
    wheel_rpm.second = toRPM(wheel_speed.second) * 1000 / config_.MAX_RPM;

    writeVelocity(0,wheel_rpm.first);
    writeVelocity(1,wheel_rpm.second);

    if ( abs(wheel_rpm.first) < 0.0001 ){
        stopMotor(0);
    }

    if ( abs(wheel_rpm.second) < 0.0001 ){
        stopMotor(1);
    }    

}