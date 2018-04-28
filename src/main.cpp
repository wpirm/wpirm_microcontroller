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

//Hardware Pin Definitions
#define HWSERIAL Serial1

#define ENCODER_R_A 30
#define ENCODER_R_B 29
#define ENCODER_L_A 25
#define ENCODER_L_B 24

#define BUMPER 33

//PID Setup
int kP = 0.15;
int kI = 0.0;
int kD = 0.0;

PID rPid = PID(kP, kI, kD);
PID lPid = PID(kP, kI, kD);

// Odometry setup (meters)
#define WHEEL_BASE 0.13
#define WHEEL_RADIUS 0.06

double x = 0.0;
double y = 0.0;
double theta = 0.0;

char base_link[] = "/base_link";
char odom[] = "/odom";

// Encoder setup

#define TICKS_PER_REV 3600.0

Encoder leftEncoder(ENCODER_L_A, ENCODER_L_B);
Encoder rightEncoder(ENCODER_R_A, ENCODER_R_B);

long lastTime = 0;
double th = 0.0;
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
    
}

// TODO: (IMPORTANT!) Redo to use delta everthing
void calculate_odom() {
    long L_ticks, R_ticks;
    double left_meters, right_meters;

    long curTime = millis();

    R_ticks = leftEncoder.read();
    leftEncoder.write(0);
    L_ticks = -rightEncoder.read();
    rightEncoder.write(0);

    left_meters = ticks_to_meters(L_ticks);
    right_meters = ticks_to_meters(R_ticks);

    
    double dt = (double)curTime/1000 - (double)lastTime/1000;

    double dxy = (double)(left_meters + right_meters) / 2.0;
    double dth = (double)(right_meters - left_meters) / (double)WHEEL_BASE;

    th += dth;
    x += dxy * cos(th);
    y += dxy * sin(th);
    evth = dth/dt;
    evx = dxy/dt;

    lastTime = curTime;
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
    // nh.loginfo("Got a cmd_vel");
    mvr = (msg.angular.z * WHEEL_BASE) / 2.0 + msg.linear.x;
	mvl = msg.linear.x * 2.0 - mvr;
}

// ROS subscribers and transform publishers
geometry_msgs::TransformStamped t;
nav_msgs::Odometry o;

tf::TransformBroadcaster broadcaster;

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);
ros::Publisher odomPub("odom", &o);
ros::Publisher bumperPub("bumper", )

// Tf broadcaster
void transform_broadcaster() {
    calculate_odom();

    // tf odom->base_link
    t.header.frame_id = "base_link";
    o.header.frame_id = "base_link";
    t.child_frame_id = "odom";
    o.child_frame_id = "odom";

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);
    
    t.transform.translation.x = x;
    t.transform.translation.y = y;

    o.pose.pose.position.x = x;
    o.pose.pose.position.y = y;
    o.pose.pose.position.z = 0.0;
    o.pose.pose.orientation = odom_quat;

    o.twist.twist.linear.x = 0.0;
    o.twist.twist.linear.y = 0.0;
    o.twist.twist.linear.z = 0.0;
    
    o.twist.twist.angular.x = 0.0;
    o.twist.twist.angular.y = 0.0;
    o.twist.twist.angular.z = 0.0;
    
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
