#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include "PID.h"

#define HWSERIAL Serial2

// PID setup (kp, ki, kd)

PID rPid = PID(0.1, 0.01, 0);
PID lPid = PID(0.1, 0.01, 0);

// Odometry setup (meters)

#define WHEEL_BASE 0.18
#define WHEEL_RADIUS 0.06

double x = 0.0;
double y = 0.0;
double theta = 0.0;

char base_link[] = "/base_link";
char odom[] = "/odom";

// Encoder setup

#define ENCODER_R_A 23
#define ENCODER_R_B 22
#define ENCODER_L_A 21
#define ENCODER_L_B 20
#define TICKS_PER_REV 6000.0

Encoder leftEncoder(ENCODER_L_A, ENCODER_L_B);
Encoder rightEncoder(ENCODER_R_A, ENCODER_R_B);

long lastTime = 0;
double total_xy = 0.0;
double total_th = 0.0;
double last_xy = 0.0;
double last_th = 0.0;

// Current encoder vels 
double evx = 0.0;
double evth = 0.0;

// Motor set vels
double mvr = 0.0;
double mvl = 0.0;

double ticks_to_meters(long ticks) {
    // Circumference * number of rotations
    return 2.0*PI*WHEEL_RADIUS*((double)ticks/TICKS_PER_REV);
}

// TODO: Finish this returns: evl and evr
void calculate_vels() {
    long curTime = millis();
    double dt = (double)curTime/1000 - (double)lastTime/1000;
    double dth = last_th - theta;
    double dxy = total_xy - last_xy;

    evx = dxy/dt;
    evth = dth/dt;

    lastTime = curTime;
    last_xy = total_xy;
    last_th = theta;
}

// TODO: (IMPORTANT!) Redo to use delta everthing
void calculate_odom() {
    long L_ticks, R_ticks;
    double left_meters, right_meters;

    R_ticks = leftEncoder.read();
    L_ticks = -rightEncoder.read();

    left_meters = ticks_to_meters(L_ticks);
    right_meters = ticks_to_meters(R_ticks);

    total_xy = (double)(left_meters + right_meters) / 2.0;

    theta = (right_meters - left_meters) / (double)WHEEL_BASE;
    theta -= (double)((int)(theta/(PI * 2.0)))*(PI * 2.0);

    x = total_xy * cos(theta); 
    y = total_xy * sin(theta); 

    calculate_vels();
}

void set_motor(double motor, double speed) {
    if(motor == 0) {
        if (speed < 0) {
            HWSERIAL.write(0xC5);
            HWSERIAL.write((byte)abs(speed));
        } 
        else {
            HWSERIAL.write(0xC6);
            HWSERIAL.write((byte)speed);
        }
    }
    else if(motor == 1) {
        if (speed < 0) {
            HWSERIAL.write(0xCD);
            HWSERIAL.write((byte)abs(speed));
        } 
        else {
            HWSERIAL.write(0xCE);
            HWSERIAL.write((byte)speed);
        }
    }
}

void set_motors() {
    double rOut = rPid.getOutput();
    double lOut = lPid.getOutput();

    set_motor(0, mvr*127);
    set_motor(1, mvl*127);
}

// ROS setup

ros::NodeHandle  nh;

void cmd_vel_cb(const geometry_msgs::Twist& msg) {
    mvr = (msg.angular.z * WHEEL_BASE) / 2.0 + msg.linear.x;
	mvl = msg.linear.x * 2.0 - mvr;
}

// ROS subscribers and transform publishers
geometry_msgs::TransformStamped t;
nav_msgs::Odometry o;

tf::TransformBroadcaster broadcaster;

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);
ros::Publisher odomPub("odom", &o);

// Tf broadcaster
void transform_broadcaster() {
    calculate_odom();

    // tf odom->base_link
    t.header.frame_id = "base_link";
    o.header.frame_id = "odom";
    t.child_frame_id = "odom";
    o.child_frame_id = "base_link";

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta);
    
    t.transform.translation.x = x;
    t.transform.translation.y = y;

    o.pose.pose.position.x = x;
    o.pose.pose.position.y = y;
    o.pose.pose.position.z = 0.0;
    o.pose.pose.orientation = odom_quat;

    o.twist.twist.linear.x = evx;
    o.twist.twist.linear.y = 0.0;
    o.twist.twist.linear.z = 0.0;
    
    o.twist.twist.angular.x = 0.0;
    o.twist.twist.angular.y = 0.0;
    o.twist.twist.angular.z = evth;
    
    t.transform.rotation = odom_quat;
    t.header.stamp = nh.now();
    o.header.stamp = nh.now();
    
    odomPub.publish(&o);
    broadcaster.sendTransform(t);
}

void setup() {
    nh.initNode();
    nh.advertise(odomPub);
    nh.subscribe(sub);
    broadcaster.init(nh);

    HWSERIAL.begin(19200);

    // PID set params
    rPid.setOutputLimits(-100,100);
	rPid.setOutputRampRate(10);
    lPid.setOutputLimits(-100,100);
	lPid.setOutputRampRate(10);
}

void loop() {
    transform_broadcaster();
    set_motors();
    nh.spinOnce();
    delay(10);
}
