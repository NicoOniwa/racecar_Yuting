/* ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu)  Development of the web-server app Dator was
# based on an open source project by Bruce Wootton, with contributions from
# Kiet Lam (kiet.lam@berkeley.edu). The RC Input code was based on sample code
# from http://rcarduino.blogspot.com/2012/04/how-to-read-multiple-rc-channels-draft.html
# --------------------------------------------------------------------------- */


// include libraries
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <EnableInterrupt.h>

/**************************************************************************
CAR CLASS DEFINITION (would like to refactor into car.cpp and car.h but can't figure out arduino build process so far)
**************************************************************************/
class Car {
  public:
    void initEncoders();
    // Used for copying variables shared with interrupts to avoid read/write
    // conflicts later
    void readAndCopyInputs();
    // Getters
    int getEncoderFL();
    int getEncoderFR();
    int getEncoderBL();
    int getEncoderBR();
    float getVelEstFL();
    float getVelEstFR();
    float getVelEstBL();
    float getVelEstBR();

    // Interrupt service routines
    void incFR();
    void incFL();
    void incBR();
    void incBL();
    void calcVelocityEstimate();
  private:
    // Pin assignments
    const uint8_t ENC_FL_PIN = 2;
    const uint8_t ENC_FR_PIN = 3;
    const uint8_t ENC_BL_PIN = 5;
    const uint8_t ENC_BR_PIN = 6;

    // Car properties
    // unclear what this is for
    const uint8_t noAction = 0;


    // Utility variables to handle RC and encoder inputs
    volatile uint8_t updateFlagsShared;
    uint8_t updateFlags;
    const uint8_t FL_FLAG = 3;
    const uint8_t FR_FLAG = 4;
    const uint8_t BL_FLAG = 5;
    const uint8_t BR_FLAG = 6;

    // Number of encoder counts on tires
    // count tick on {FL, FR, BL, BR}
    // F = front, B = back, L = left, R = right
    volatile uint32_t FL_count_shared = 0;
    volatile uint32_t FR_count_shared = 0;
    volatile uint32_t BL_count_shared = 0;
    volatile uint32_t BR_count_shared = 0;
    uint32_t FL_count = 0;
    uint32_t FR_count = 0;
    uint32_t BL_count = 0;
    uint32_t BR_count = 0;
    uint32_t FL_count_old = 0;
    uint32_t FR_count_old = 0;
    uint32_t BL_count_old = 0;
    uint32_t BR_count_old = 0;
    float vel_FL = 0;
    float vel_FR = 0;
    float vel_BL = 0;
    float vel_BR = 0;


    // Timing parameters
    // F = front, B = back, L = left, R = right   
    volatile unsigned long FL_new_time = 0;
    volatile unsigned long FR_new_time = 0;
    volatile unsigned long BL_new_time = 0;
    volatile unsigned long BR_new_time = 0;
    volatile unsigned long FL_old_time = 0; 
    volatile unsigned long FR_old_time = 0;
    volatile unsigned long BL_old_time = 0;
    volatile unsigned long BR_old_time = 0;
    unsigned long FL_DeltaTime = 0;
    unsigned long FR_DeltaTime = 0;
    unsigned long BL_DeltaTime = 0;
    unsigned long BR_DeltaTime = 0;
};

// Boolean keeping track of whether the Arduino has received a signal from the ECU recently

const float pi = 3.141593;
const float R = 0.0235;        // radius of the wheel

// Initialize an instance of the Car class as car
Car car;

// Callback Functions
// These are really sad solutions to the fact that using class member functions
// as callbacks is complicated in C++ and I haven't figured it out. If you can
// figure it out, please atone for my sins.

void incFLCallback() {
  car.incFL();
}
void incFRCallback() {
  car.incFR();
}
void incBLCallback() {
  car.incBL();
}
void incBRCallback() {
  car.incBR();
}

// Variables for time step
volatile unsigned long dt;
volatile unsigned long t0;
volatile unsigned long led_t0;
volatile unsigned long vel_t0;      //velocity estimation timer

// Global message variables
// Encoder, RC Inputs, Electronic Control Unit, Ultrasound

std_msgs::Float32MultiArray encoder_msg;
std_msgs::Float32MultiArray vel_est_msg;

ros::NodeHandle nh;

ros::Publisher pub_encoder("encoder", &encoder_msg);
ros::Publisher pub_vel_est("vel_est", &vel_est_msg); 


/**************************************************************************
ARDUINO INITIALIZATION
**************************************************************************/
void setup()
{
  // Set up encoders, rc input, and actuators
  car.initEncoders();

  // Start ROS node
  nh.initNode();

// initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // Publish and subscribe to topics
  nh.advertise(pub_encoder);
  nh.advertise(pub_vel_est);

  // Initialize encoder and velocity estimate messages
  encoder_msg.data_length = 4;
  static float encoder_data[4];
  encoder_msg.data = encoder_data;
  vel_est_msg.data_length = 4;
  static float vel_est_data[4];
  vel_est_msg.data = vel_est_data;

  // Arming ESC, 1 sec delay for arming and ROS
  t0 = millis();
  led_t0 = millis();
  

}


/**************************************************************************
ARDUINO MAIN lOOP
**************************************************************************/
void loop() {
  // compute time elapsed (in ms)
  dt = millis() - t0;

  // kill the motor if there is no ECU signal within the last 1s

  if (dt > 50) {
    car.readAndCopyInputs();

    // publish velocity estimate
    pub_vel_est.publish(&vel_est_msg); 

    // publish encoder ticks
    encoder_msg.data[0] = car.getEncoderFL();
    encoder_msg.data[1] = car.getEncoderFR();
    encoder_msg.data[2] = car.getEncoderBL();
    encoder_msg.data[3] = car.getEncoderBR();
    pub_encoder.publish(&encoder_msg);

    t0 = millis();
  }

  if (millis() - vel_t0 >=100) {
    car.calcVelocityEstimate();
    vel_est_msg.data[0] = car.getVelEstFL();
    vel_est_msg.data[1] = car.getVelEstFR();
    vel_est_msg.data[2] = car.getVelEstBL();
    vel_est_msg.data[3] = car.getVelEstBR();
    vel_t0 = millis();
  }

  //led blink logic
  if (millis() - led_t0 >=500) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    led_t0 = millis();
  }

  nh.spinOnce();
}

/**************************************************************************
CAR CLASS IMPLEMENTATION
**************************************************************************/

void Car::initEncoders() {
  pinMode(ENC_FR_PIN, INPUT_PULLUP);
  pinMode(ENC_FL_PIN, INPUT_PULLUP);
  pinMode(ENC_BR_PIN, INPUT_PULLUP);
  pinMode(ENC_BL_PIN, INPUT_PULLUP);
  enableInterrupt(ENC_FR_PIN, incFRCallback, CHANGE);
  enableInterrupt(ENC_FL_PIN, incFLCallback, CHANGE);
  enableInterrupt(ENC_BR_PIN, incBRCallback, CHANGE);
  enableInterrupt(ENC_BL_PIN, incBLCallback, CHANGE);
}

void Car::incFL() {
  FL_count_shared++;
  FL_old_time = FL_new_time; 
  FL_new_time = micros();
  updateFlagsShared |= FL_FLAG;
}

void Car::incFR() {
  FR_count_shared++;
  FR_old_time = FR_new_time;
  FR_new_time = micros();
  updateFlagsShared |= FR_FLAG;
}

void Car::incBL() {
  BL_count_shared++;
  BL_old_time = BL_new_time;
  BL_new_time = micros();
  updateFlagsShared |= BL_FLAG;
}

void Car::incBR() {
  BR_count_shared++;
  BR_old_time = BR_new_time;
  BR_new_time = micros();
  updateFlagsShared |= BR_FLAG;
}

void Car::readAndCopyInputs() {
  // check shared update flags to see if any channels have a new signal
  if (updateFlagsShared) {
    // Turn off interrupts, make local copies of variables set by interrupts,
    // then turn interrupts back on. Without doing this, an interrupt could
    // update a shared multibyte variable while the loop is in the middle of
    // reading it
    noInterrupts();
    // make local copies
    updateFlags = updateFlagsShared;
   
    if(updateFlags & FL_FLAG) {
      FL_count = FL_count_shared;
      FL_DeltaTime = FL_new_time - FL_old_time;
    }
    if(updateFlags & FR_FLAG) {
      FR_count = FR_count_shared;
      FR_DeltaTime = FR_new_time - FR_old_time;
    }
    if(updateFlags & BL_FLAG) {
      BL_count = BL_count_shared;
      BL_DeltaTime = BL_new_time - BL_old_time;
    }
    if(updateFlags & BR_FLAG) {
      BR_count = BR_count_shared;
      BR_DeltaTime = BR_new_time - BR_old_time;
    }
    // clear shared update flags and turn interrupts back on
    updateFlagsShared = 0;
    interrupts();
  }
}

int Car::getEncoderFL() {
  return FL_count;
}
int Car::getEncoderFR() {
  return FR_count;
}
int Car::getEncoderBL() {
  return BL_count;
}
int Car::getEncoderBR() {
  return BR_count;
}
float Car::getVelEstFL() {
  return vel_FL;
}
float Car::getVelEstFR() {
  return vel_FR;
}
float Car::getVelEstBL() {
  return vel_BL;
}
float Car::getVelEstBR() {
  return vel_BR;
}

void Car::calcVelocityEstimate() {

    // vel = distance / time
    // distance = 2*pi*R/4 since there are 4 partitions,
    if(FL_count_old != FL_count){
        vel_FL = 0.5*pi*R/(FL_DeltaTime/1000000.0);    }
    else{ vel_FL = 0.0; }

    if(FR_count_old != FR_count){
        vel_FR = 0.5*pi*R/(FR_DeltaTime/1000000.0);    }
    else{ vel_FR = 0.0; }

    if(BL_count_old != BL_count){
        vel_BL = 0.5*pi*R/(BL_DeltaTime/1000000.0);    }
    else{ vel_BL = 0.0; }

    if(BR_count_old != BR_count){
        vel_BR = 0.5*pi*R/(BR_DeltaTime/1000000.0);    }
    else{ vel_BR = 0.0; }

    // update history
    FL_count_old = FL_count;
    FR_count_old = FR_count;
    BL_count_old = BL_count;
    BR_count_old = BR_count;
}
