#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <AccelStepper.h>

#define MotorInterfaceType 1

const int stepPin = 2;  
const int dirPin = 3;   
const int stepPin2 = 4; 
const int dirPin2 = 5; 
const int stepPin3 = 6;  
const int dirPin3 = 7;   
const int stepPin4 = 8;  
const int dirPin4 = 9;   

AccelStepper Base(MotorInterfaceType, stepPin, dirPin);
AccelStepper Shoulder(MotorInterfaceType, stepPin2, dirPin2);
AccelStepper elbow(MotorInterfaceType, stepPin3, dirPin3);
AccelStepper LowerWrist(MotorInterfaceType, stepPin4, dirPin4);


// Create ROS node handle
ros::NodeHandle nh;

// Global variables

float base_ang;
float shoulder_ang;
float uwrist_ang;
float elbow_ang;
float lwrist_ang;

float convert_to_step(float angle) {
  double degree = angle * 57.32;
  float steps = degree * 900;
  return steps;


}

void messageCb(const sensor_msgs::JointState& angles) {

  base_ang = convert_to_step(angles.position[0]);
  shoulder_ang = convert_to_step(angles.position[1]);
  elbow_ang = convert_to_step(angles.position[2]);
  lwrist_ang = convert_to_step(angles.position[3]);
  uwrist_ang = convert_to_step(angles.position[4]);

}

// ROS subscriber and publisher
ros::Subscriber<sensor_msgs::JointState> sub("JointState", messageCb);


void setup() {
  pinMode(13, OUTPUT);
  nh.initNode();        // Initialize ROS node
  nh.subscribe(sub);

  Base.setMaxSpeed(1100);
  Shoulder.setMaxSpeed(1100);
  elbow.setMaxSpeed(1100);
  LowerWrist.setMaxSpeed(1100);
  Base.setAcceleration(900);
  Shoulder.setAcceleration(900);
  elbow.setAcceleration(900);
  LowerWrist.setAcceleration(900);
  digitalWrite(13, HIGH);

  Serial.begin(57600);
      // Subscribe to the topic
}

void loop() {
  nh.spinOnce();  // Process ROS messages

  Base.moveTo(base_ang);
  Shoulder.moveTo(shoulder_ang);

  elbow.moveTo(elbow_ang);
  LowerWrist.moveTo(lwrist_ang);

  // Run each motor until they reach their target positions
  Base.run();
  Shoulder.run();
  elbow.run();
  LowerWrist.run();
  digitalWrite(13, HIGH);
  


  nh.spinOnce();

  }

  //to do 
  //encoder integration
  //fix gearbox T^T
