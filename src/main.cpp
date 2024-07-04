/*
  Simple Motion Algorithm for Quadrupeds, by Yunus Zenichowski
  Be warned, the code is very bad. There is plenty of room for optimization and
  probably a ton of bugs. However, I think it should work. made for Arduino
  (ESP32)

  In case you found a bug or have questions, please contact me:
  y.zenichowski@gmail.com

  IMPORTANT! PLEASE READ:
  1) You will need to change the adress of for your ps 3 controller, look in the
  setup() function down below 2) Read the Comment above the servo_mask()
  function and follow the instructions! You will also find it somewhere in the
  middle of the code    <-- VERY IMPORTANT!!!!

  The coordinate system used in the agorithm:

  view on right side                      view on BACK
  -------→X        |------------|         ------→Y        _ _-------_ _
  |                /\ ________/\_|        |               |_|-_______-|_|
  |               / /        / /          |                 |         |
  |               \/         \/           |                 |         |
  ↓Z               \          \           ↓Z                |         |
  \          \                            |         |


  front-left leg   =  leg 0
  back-left leg    =  leg 1
  front-right leg  =  leg 2
  back-right leg   =  leg 3

  CONTROLLS:
  Left Stick:
  forward/backwad, left/right

  Right Stick:
  turn left/turn right, up/down

  Cross Button:
  STEP_HEIGHT set to -0.2
  Circle Button:
  STEP_HEIGHT set to -0.3
  Triangle Button:
  STEP_HEIGHT set to -0.4
  Square Button:
  STEP_HEIGHT set to -0.5

  Down Button:
  MOTION_RES set to 16
  Right Button:
  MOTION_RES set to 18
  Up Button:
  PUSH_HEIGHT set to 0.1
  Left Button:
  PUSH_HEIGHT set to 0.0

  Explanation of the variables down below
*/

#include <cstring>
#include <cstdint>
#include <cmath>

#if !ARDUINO_FREE
#  include <Ps3Controller.h>
#  include <Adafruit_PWMServoDriver.h>
#else
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <sstream>

/* arduino specific functionality */

// source: https://www.arduino.cc/reference/en/language/functions/math/map/
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

struct {
    template<typename T> void print(T arg) { *output << arg; }
    template<typename T> void println(T arg) { *output << arg << '\n'; }
    void println() { *output << '\n'; }

    void change_ostream(std::ostream* output) {this->output = output;}

private:
    std::ostream* output = &std::cout;
} Serial;

void delay(uint64_t ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

/* hardware library functionality */

struct Adafruit_PWMServoDriver {

    Adafruit_PWMServoDriver(unsigned int){}

    uint8_t setPWM(uint8_t num, uint16_t on, uint16_t off)
    {
	return 0;
    }
};

struct {
    struct {
	struct {
	    struct {
		double lx = 0, ly = 0;
		double rx = 0, ry = 0;
	    } stick;
	} analog;
    } data;
    
    struct {
	struct {
	    bool cross = false;
	    bool circle = false;
	    bool triangle = false;
	    bool square = false;
	    bool down = false;
	    bool right = false;
	    bool up = false;
	    bool left = false;
	} button_up;
    } event;
} Ps3;

#endif // ARDUINO

using namespace std;

// SERVO PARAMS

// Define maximum and minimum number of "ticks" for the servo motors.          /* ! experimenting reqiered ! */
// Range from 0 to 4095. This determines the pulse width.
constexpr uint32_t SERVOMIN = 100;
constexpr uint32_t SERVOMAX = 550;
 
// Servo motor indices -> translation from the Servo Driver to the internal ordering.
const int SERVO_CONNECTIONS[12] = {11, 10, 9, 15, 14, 13, 4, 5, 6, 0, 1, 2};

// SIMPLE MOTION ALORITHM CONSTANTS START

#define NUMBER_OF_LEGS 4
#define SERVOS_PER_LEG 3

// The QP_SIZE can be set to anything. For example Increasing the Y SIZE, increases the distance
// between the left and right legs. Increasing X SIZE will increase the distance between the front and rear legs.
// ACTUAL_QP_SIZE is needed for the computation, and should represent the exact distances
// between the hip joints (Y) and the pivots of the upper legs (X)
constexpr float QP_SIZEX = 220;
constexpr float QP_SIZEY = 200;
constexpr float ACTUAL_QP_SIZEX = 218;
constexpr float ACTUAL_QP_SIZEY = 110;

// These are just relevant for visualization
constexpr float SERVO_0_AXIS_OFFSET = 21.2;
constexpr float LEG_WIDTH_OFFSET = 7.5;

constexpr float UPLEGLGTH = 130;  // UPLEGLGTH is the length of the upper leg in mm
constexpr float LOWLEGLGTH = 130; // LOWLEGLGTH is the length of the lower leg in mm

#define LEG_OFFSET_Y 32.4  // This is the Y distance between the hip joint and the leg tip

// Center of gravity offset in mm from the midpoint of the quadruped
#define COG_X_OFFSET 0
#define COG_Y_OFFSET 0

// The MIN / MAX variables are required, if the algorithm should adaptively change the parameters depending on speed etc.
// However this is an experimental feature that I think just makes things worse
#define MAX_X_DIR 80
#define MAX_Y_DIR 50
#define START_PAIR 0 // 0:(leg0, leg3) or 1:(leg1, leg2) will be the first "step_pair"
#define WALK_RUN_PIVOT 0.8 // of max speed, is the point for the transition from walking to running
#define MAX_MOTION_RES 20 // MOTION_RES varies with speed
#define MIN_MOTION_RES 15
#define MAX_STEP_TIME_REDUCTION 3
#define MIN_STEP_TIME_REDUCTION 1
#define MAX_PUSH_HEIGHT 0 //0
#define MAX_DIR_VEC_LENGTH sqrt(sqr(MAX_X_DIR) + sqr(MAX_Y_DIR))
#define MAX_X_BODY_OFFSET_RATIO 5 // meaning 1/5
#define MAX_Y_BODY_OFFSET_RATIO 5 // meaning 1/5

#if VISUALIZE_QUADRUPED
#include "visualize_quadruped.hpp"
QP_Viewport qp_viewport{ACTUAL_QP_SIZEX, ACTUAL_QP_SIZEY,     UPLEGLGTH,
                        LOWLEGLGTH,      SERVO_0_AXIS_OFFSET, LEG_WIDTH_OFFSET};
bool exit_graphical_environment = false;
#endif // VISUALIZE_QUADRUPED

// mathematical

constexpr double EPSILON = 0.000000001;
constexpr double qp_PI = 3.14159265359;
constexpr double qp_RAD_TO_DEG = 180.0 / qp_PI;
constexpr double qp_DEG_TO_RAD = qp_PI / 180.0;

#define IDX_2D(x, y, nx) ((x) + (y) * (nx))

template<typename T> struct vec2 {
    T x, y;
    T& operator[](size_t idx){return *(&x + idx);}
    const T& operator[](size_t idx) const {return *(&x + idx);}
    vec2<T> operator/(T s) const { return {x/s, y/s}; }
    vec2<T> operator*(T s) const { return {x*s, y*s}; }
};

template<typename T> struct vec3 {
    T x, y, z;
    T& operator[](size_t idx){return *(&x + idx);}
    const T& operator[](size_t idx) const {return *(&x + idx);}
    vec3<T> operator/(T s) const { return {x/s, y/s, z/s}; }
    vec3<T> operator*(T s) const { return {x*s, y*s, z*s}; }
};

template<typename T> struct vec4 {
    T x, y, z, a;
    T& operator[](size_t idx){return *(&x + idx);}
    const T& operator[](size_t idx) const { return *(&x + idx); }
    vec4<T> operator/(T s) const { return {x/s, y/s, z/s, a/s}; }
    vec4<T> operator*(T s) const { return {x*s, y*s, z*s, a*s}; }
};

// SIMPLE MOTION ALORITHM CONSTANTS END



// SIMPLE MOTION ALORITHM GLOBAL VARIABLES START
// Normal height of the Quadruped
float HOME_HEIGHT = 170;
// The resolution per step, can be any even number (maybe also odd).
// Lower resolution allows for faster compute and therfore execute time.
float MOTION_RES = 18; // positive integer >= 1
// This is just needed for safety
float new_MOTION_RES = MOTION_RES;
// STEP_HEIGHT of -0.5 means, that the step arc is computed using a point with Z = (Height_of_quadruped + (Height_of_quadruped * -0.5))
float STEP_HEIGHT = -0.4; // min -1 max 0
float new_STEP_HEIGHT = STEP_HEIGHT;
// This is the opposite of STEP_HEIGHT. Its used to cancel out the slight drop of the body due to the lifting of the "step_pair" during the step.
float PUSH_HEIGHT = 0.1; // min 0 max 1
float new_PUSH_HEIGHT = PUSH_HEIGHT;
float STEP_TIME_REDUCTION = 1.5; // positive integer >= 1
// The implementation of smoothe accelleration is based on delay(), so I recommend to leave this at 0.
float STEP_ACCELERATION_WEIGHT = 0;


// These are the Positions of the leg tips in their "home" position in realtion to the global coordinate system
vec3<float> home_leg_pos[NUMBER_OF_LEGS] = {
    {QP_SIZEX / 2.0 + COG_X_OFFSET,  -ACTUAL_QP_SIZEY / 2.0 + COG_Y_OFFSET, HOME_HEIGHT},
    {-QP_SIZEX / 2.0 + COG_X_OFFSET, -ACTUAL_QP_SIZEY / 2.0 + COG_Y_OFFSET, HOME_HEIGHT},
    {QP_SIZEX / 2.0 + COG_X_OFFSET,  ACTUAL_QP_SIZEY / 2.0 + COG_Y_OFFSET,  HOME_HEIGHT},
    {-QP_SIZEX / 2.0 + COG_X_OFFSET, ACTUAL_QP_SIZEY / 2.0 + COG_Y_OFFSET,  HOME_HEIGHT}
};

// Also needed for the "Actual" quadruped size.
const vec3<float> actual_home_leg_pos[NUMBER_OF_LEGS] = {
    {ACTUAL_QP_SIZEX / 2.0,  -QP_SIZEY / 2.0, HOME_HEIGHT},
    {-ACTUAL_QP_SIZEX / 2.0, -QP_SIZEY / 2.0, HOME_HEIGHT},
    {ACTUAL_QP_SIZEX / 2.0,  QP_SIZEY / 2.0,  HOME_HEIGHT},
    {-ACTUAL_QP_SIZEX / 2.0, QP_SIZEY / 2.0,  HOME_HEIGHT}
};

// This is where the current leg position is stored. (very important variable)
vec3<float> glob_leg_pos[NUMBER_OF_LEGS] = {
    home_leg_pos[0],
    home_leg_pos[1],
    home_leg_pos[2],
    home_leg_pos[3]
};

float get_angle(float x, float y) {
    return std::atan2(y, x) * qp_RAD_TO_DEG + ((y <= 0) * 360);
}

// these are the angles of the vectors to the leg tips in their "home" position

// in case the IK algorithm cant find a solution, the previous angles will be used
vec3<float> previous_angles[NUMBER_OF_LEGS] = {
    { 90, 90, 90 },
    { 90, 90, 90 },
    { 90, 90, 90 },
    { 90, 90, 90 }
};

// The previous step pair. 
int prev_step_pair = int(!START_PAIR);

// Here the two last directions (global direction of the quadruped) are saved
vec4<float> prev_dir[2] = { { 0,0,0,0 }, {0,0,0,0} };

// for the servo communication
//double pwm0, pwm1, pwm2, pwm3, pwm4, pwm5, pwm6, pwm7, pwm8, pwm9, pwm10, pwm11;
//vector<double> PWM(12);
//assigning I2C address to pca9685
Adafruit_PWMServoDriver pca9685{0x40};
//const int MPU_ADDR = 0x68;

// Input directions. These define the next step and can be controlled using the the ps 3 controller.
// For example a x_direction of 60 means the Quadruped will move 60 mm in one step in the X direction of its coordintae system.
double x_direction = 0;
double y_direction = 0;
double z_direction = 0;
double rotation_direction = 0; // in degrees

// SIMPLE MOTION ALORITHM GLOBAL VARIABLES END

double sqr(double x) { return(x * x); }

// SIMPLE MOTION ALGORITHM START!
// angle between two 2d vectors
double angle_vec2_vec2(double a_x, double a_y, double b_x, double b_y) {
    return (acos((a_x * b_x + a_y * b_y) / (sqrt(sqr(a_x) + sqr(a_y)) * sqrt(sqr(b_x) + sqr(b_y))))) * double(qp_RAD_TO_DEG);
}

// Inverse Kinematics Algorithm, might be buggy
// pos = (x,y,z) in regards to the local coordinate system of the leg
// leg = id of the leg
vec3<float> get_IK(float x, float y, float z, int leg)
{
    double knee_x, knee_y;
    float angle_x, angle_y, angle_z;
    double trans_pos_x, trans_pos_y, trans_pos_z;
    double l1 = sqrt(sqr(y) + sqr(z));
    double C = (qp_PI / 2.0 - std::asin(LEG_OFFSET_Y / l1)) * qp_RAD_TO_DEG;
    double Cdiff = 90.0 - C;
    if (leg >= 2)
	Cdiff *= -1;
    double l3 = l1 * sin(C * qp_DEG_TO_RAD);
    trans_pos_x = x;
    trans_pos_y = (l3 / sqrt(sqr(y) + sqr(z))) * y;
    trans_pos_z = (l3 / sqrt(sqr(y) + sqr(z))) * z;
    double test = sqrt(sqr(trans_pos_y) + sqr(trans_pos_z));
    trans_pos_y = trans_pos_y * cos(Cdiff * qp_DEG_TO_RAD) - trans_pos_z * sin(Cdiff * qp_DEG_TO_RAD);
    trans_pos_z = trans_pos_y * sin(Cdiff * qp_DEG_TO_RAD) + trans_pos_z * cos(Cdiff * qp_DEG_TO_RAD);
    double test1 = sqrt(sqr(trans_pos_y) + sqr(trans_pos_z));
    trans_pos_y = (l3 / sqrt(sqr(trans_pos_y) + sqr(trans_pos_z))) * trans_pos_y;
    trans_pos_z = (l3 / sqrt(sqr(trans_pos_y) + sqr(trans_pos_z))) * trans_pos_z;
    //trans_pos_y = -trans_pos_y; // the trans_pos_y also needs to be flipt, since trans_pos_z is already flipped
    double test2 = sqrt(sqr(trans_pos_y) + sqr(trans_pos_z));
    // extending the length of the xz vector to the length of the xyz vector
    double q1 = sqrt((sqr(trans_pos_x) + sqr(trans_pos_y) + sqr(trans_pos_z)) / (sqr(trans_pos_x) + sqr(trans_pos_z) + EPSILON));
    trans_pos_x *= q1;
    trans_pos_z *= q1;

    // Calculation of the intersection points of the circles, created by the radii of the legs (Source: http://paulbourke.net/geometry/circlesphere/tvoght.c)
    double x0 = 0, y0 = 0, x1 = trans_pos_x, y1 = trans_pos_z; // mid points of the two circles
    double r0 = UPLEGLGTH, r1 = LOWLEGLGTH;
    double dx = x0 - x1;
    double dy = y0 - y1;
    double d = hypot(dx, dy) + EPSILON;
    double a = ((r0 * r0) - (r1 * r1) + (d * d)) / (2.0 * d);
    double x2 = x0 + (dx * a / d);
    double y2 = y0 + (dy * a / d);
    double h0 = sqrt((r0 * r0) - (a * a));
    double rx = -dy * (h0 / d);
    double ry = dx * (h0 / d);
    // Determine the absolute intersection points. Just using one of the two points.
    double fx = -(x2 + rx);
    double fy = -(y2 + ry);
    knee_x = fx;
    knee_y = fy;
    // calculating the angles between the computed vectors and the vectors representing the minimal servo position
    angle_x = float(angle_vec2_vec2(1.0, 0, trans_pos_y, trans_pos_z)); // 0 // (leg >= 2 ? 1.0 : -1.0)
    angle_y = float(angle_vec2_vec2(1.0, 1.0, knee_x, knee_y)); // 1
    angle_z = float(angle_vec2_vec2(knee_x - trans_pos_x, knee_y - trans_pos_z, 1,-1)); // 2

    // in case one of the angles is not a number (meaning there isnt a solution) then it returns the previous angles 
    if (angle_x + angle_y + angle_z != angle_x + angle_y + angle_z)
	return {previous_angles[leg][0], previous_angles[leg][1], previous_angles[leg][2]};
    previous_angles[leg][0] = angle_x;
    previous_angles[leg][1] = angle_y;
    previous_angles[leg][2] = angle_z;
    return {angle_x, angle_y, angle_z};
}

// servo_mask() Is used to correct for slightly incorrect leg assembly (wrong leg segment orientation) AND to take into account,
// that some servos positions have to be mirrored by subtracting the servo position from the maximum servo position (180°)
// ATTENTION!! The current values that are subtracted from the "angle" were my personal values and are there for demonstration purposes.
// You will need to change them to 0 and then after the assembly of the robot, add or subtract something in order to adjust the physical leg orientation to match the software model !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Use the tune_servo_positions() function for the assembly in order to get the orientation of them right. This function can be found close to the bottom.
float servo_mask(float angle, int servo){

    switch(servo){
	// 0
    case 3: return((angle + 22));
    case 4: return(180.0 - (angle - 38));
    case 5: return(180.0 - (angle - 29.5));
	// 1
    case 0: return(180.0 - (angle + 13));
    case 1: return(180.0 - (angle - 32));
    case 2: return(180.0 - (angle - 31));
	// 2
    case 9: return(angle - 13);
    case 10: return(angle - 38);
    case 11: return(angle - 28);
	// 3
    case 6: return(180.0 - (angle - 11));
    case 7: return(angle - 36);
    case 8: return(angle - 29);
    }
    return 0;
}

float default_servo_mask(float angle, int servo){

    switch(servo){
	// 0
    case 3: return(angle);
    case 4: return(180.0 - angle);
    case 5: return(180.0 - angle);
	// 1
    case 0: return(180 - angle);
    case 1: return(180.0 - angle);
    case 2: return(180.0 - angle);
	// 2
    case 9: return(angle);
    case 10: return(angle);
    case 11: return(angle);
	// 3
    case 6: return(180.0 - angle);
    case 7: return(angle);
    case 8: return(angle);
    }
    return 0;
}

// moves the servos to whatever is currently in glob_leg_pos
void update_servos()
{
    // Subtracting the "actual home leg position" from the global leg positions,
    // in order to move the coordinates from the global to the local coordinate system  
    vec3<float> new_leg_pos[NUMBER_OF_LEGS] = { {0,0,0},{0,0,0},{0,0,0},{0,0,0} };
    for(int i = 0; i < 4; i++){
	for(int j = 0; j < 2; j++)
	    new_leg_pos[i][j] = glob_leg_pos[i][j] - actual_home_leg_pos[i][j];
	new_leg_pos[i][2] = glob_leg_pos[i][2];
    }
  
    // Iterating over all servos and moving them to the new position
    vec3<float> new_servo_pos;
    
#if VISUALIZE_QUADRUPED
    vec3<float> vq_leg_servo_angles[4];
    qp_viewport.get_Ps3_input(&Ps3.data.analog.stick.lx, &Ps3.data.analog.stick.ly, &Ps3.data.analog.stick.rx, &Ps3.data.analog.stick.ry);
#endif
    
    for(int leg = 0; leg < NUMBER_OF_LEGS; leg++){
	new_servo_pos = get_IK(new_leg_pos[leg][0], new_leg_pos[leg][1], new_leg_pos[leg][2], leg);
	
	for(int servo = 0; servo < SERVOS_PER_LEG; ++servo){
	    // setting the servo positions
#if VISUALIZE_QUADRUPED
	    vq_leg_servo_angles[leg][servo] = new_servo_pos[servo];
#endif
	    pca9685.setPWM(SERVO_CONNECTIONS[IDX_2D(servo, leg, SERVOS_PER_LEG)],
			   0,
			   map(servo_mask(new_servo_pos[servo], IDX_2D(servo, leg, SERVOS_PER_LEG)), 0, 180, SERVOMIN, SERVOMAX));
	    // printing the servo angles in the serial monitor
	    Serial.print(SERVO_CONNECTIONS[IDX_2D(servo, leg, SERVOS_PER_LEG)]);
	    Serial.print(", ");
	    Serial.print(servo_mask(new_servo_pos[servo], SERVO_CONNECTIONS[IDX_2D(servo, leg, SERVOS_PER_LEG)]));
	    Serial.print(" | ");
	}
	Serial.print("  ");
    }
    Serial.println();
    
#if VISUALIZE_QUADRUPED
    exit_graphical_environment =
	!qp_viewport.update_viewport(Vector3{vq_leg_servo_angles[0].x, vq_leg_servo_angles[0].y, vq_leg_servo_angles[0].z},
				     Vector3{vq_leg_servo_angles[1].x, vq_leg_servo_angles[1].y, vq_leg_servo_angles[1].z},
				     Vector3{vq_leg_servo_angles[2].x, vq_leg_servo_angles[2].y, vq_leg_servo_angles[2].z},
				     Vector3{vq_leg_servo_angles[3].x, vq_leg_servo_angles[3].y, vq_leg_servo_angles[3].z});
#endif
}

// like the map() function, but for float instead of int
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

vec4<float> get_move_commandf()
{
    vec4<float> new_dir = {(float)x_direction, (float)y_direction, (float)z_direction, (float)rotation_direction};
    return(new_dir);
}

static void get_rota_trans(vec4<float>* next_step, float angle)
{
    for (int i = 0; i < 2; ++i) {
	next_step[i].x = cos(angle * float(qp_DEG_TO_RAD)) * next_step[i].x - sin(angle * float(qp_DEG_TO_RAD)) * next_step[i].y;
	next_step[i].y = sin(angle * float(qp_DEG_TO_RAD)) * next_step[i].x + cos(angle * float(qp_DEG_TO_RAD)) * next_step[i].y;
    }
}

static void get_rota_trans(vec3<float> *next_step, float angle)
{
    for (int i = 0; i < 2; ++i) {
	next_step[i].x = cos(angle * float(qp_DEG_TO_RAD)) * next_step[i].x - sin(angle * float(qp_DEG_TO_RAD)) * next_step[i].y;
	next_step[i].y = sin(angle * float(qp_DEG_TO_RAD)) * next_step[i].x + cos(angle * float(qp_DEG_TO_RAD)) * next_step[i].y;
    }
}

static void get_step(vec4<float> *step_pair, vec4<float> *push_pair, vec4<float> &next_dir)
{
    if ((signbit(prev_dir[0][0]) + signbit(next_dir[0])) == 1 ||
	(signbit(prev_dir[0][1]) + signbit(next_dir[1])) == 1 ||
	(signbit(prev_dir[0][2]) + signbit(next_dir[2])) == 1 ||
	(signbit(prev_dir[0][3]) + signbit(next_dir[3])) == 1)
	prev_step_pair = !prev_step_pair;
    if (prev_step_pair == 1) {
	step_pair[0][3] = 0;
	step_pair[1][3] = 3;
	push_pair[0][3] = 1;
	push_pair[1][3] = 2;
	prev_step_pair = 0;
    }
    else if (prev_step_pair == 0) {
	step_pair[0][3] = 1;
	step_pair[1][3] = 2;
	push_pair[0][3] = 0;
	push_pair[1][3] = 3;
	prev_step_pair = 1;
    }
    for (int i = 0; i < 2; i++)
	for (int j = 0; j < 3; j++)
	    step_pair[i][j] = home_leg_pos[int(step_pair[i][3])][j] + next_dir[j] / 2;
    get_rota_trans(step_pair, next_dir.a);
}

vec3<float> bez3_3d_at(vec3<float> &p0, vec3<float> &p1, vec3<float> &p2, float at)
{
    float p01, p12, p01_12;
    vec3<float> bez_result_at;
    for (int j = 0; j < 3; j++) {
	p01 = (p0[j] + (p1[j] - p0[j]) * at);
	p12 = (p1[j] + (p2[j] - p1[j]) * at);
	p01_12 = (p01 + (p12 - p01) * at);
	bez_result_at[j] = p01_12;
    }
    return(bez_result_at);
}

vec3<float> incl_z(vec4<float> &next_leg_pos, vec4<float> &prev_leg_pos, float z_peak, int i)
{
    vec3<float> p0 = { prev_leg_pos[0], prev_leg_pos[1], prev_leg_pos[2] };
    vec3<float> p1 = { (next_leg_pos[0] + prev_leg_pos[0]) / 2, (next_leg_pos[1] + prev_leg_pos[1]) / 2, ((next_leg_pos[2] + prev_leg_pos[2]) / 2) + ((next_leg_pos[2] + prev_leg_pos[2]) / 2) * float(z_peak) };
    vec3<float> p2 = { next_leg_pos[0], next_leg_pos[1], next_leg_pos[2] };
    return bez3_3d_at(p0, p1, p2, (i >= 0 ? i / MOTION_RES : 0));
}

void add_globLeg_stepPair(vec4<float> *step_pair, vec4<float> *step_pair_prev_pos, vec4<float>& next_dir, int i)
{
    for (int j = 0; j < 2; j++)
	glob_leg_pos[int(step_pair[j][3])] = incl_z(step_pair[j],
						    step_pair_prev_pos[j],
						    (sqrt(sqr(next_dir[0] + prev_dir[0][0] + prev_dir[1][0])
							  + sqr(next_dir[1] + prev_dir[0][1] + prev_dir[1][1])
							  + sqr(next_dir[2] + prev_dir[0][2] + prev_dir[1][2])
							  + sqr(next_dir[3] + prev_dir[0][3] + prev_dir[1][3]))
						     >= 4 ? STEP_HEIGHT : 0), i);
}

void add_globLeg_pushPair(vec4<float> *push_pair, vec4<float> *push_pair_prev_pos, vec4<float> &next_dir, int i)
{
    vec4<float> next_dir_portions = next_dir / MOTION_RES;
    vec4<float> next_dir_portions_x_i;
    for (int j = 0; j < 2; j++) {
	// assigning the targeted end position for the pushing legs
	for(int h = 0; h < 3; h++)
	    push_pair[j][h] = push_pair_prev_pos[j][h] - next_dir[h];
	// including the rotational motion
	glob_leg_pos[int(push_pair[j][3])] = incl_z(push_pair[j],
						    push_pair_prev_pos[j],
						    (sqrt(sqr(next_dir[0] + prev_dir[0][0] + prev_dir[1][0]) +
							  sqr(next_dir[1] + prev_dir[0][1] + prev_dir[1][1]) +
							  sqr(next_dir[2] + prev_dir[0][2] + prev_dir[1][2]) +
							  sqr(next_dir[3] + prev_dir[0][3] + prev_dir[1][3]))
						     >= 4 ? PUSH_HEIGHT : 0), i);
	next_dir_portions_x_i = next_dir_portions * -i; // we need a minus, since the pushing pair has to go in the reversed direction of the global direction
    }
    vec3<float> next_step[2] = { glob_leg_pos[int(push_pair[0][3])], glob_leg_pos[int(push_pair[1][3])] };
    get_rota_trans(next_step, next_dir_portions_x_i.a);
    for (int j = 0; j < 2; j++)
	for (int h = 0; h < 2; h++)
	    glob_leg_pos[int(push_pair[j][3])][h] = next_step[j][h];
}

void simple_motion_func()
{
    PUSH_HEIGHT = new_PUSH_HEIGHT;
    STEP_HEIGHT = new_STEP_HEIGHT;
    MOTION_RES = new_MOTION_RES;
    for (int i = 0; i < 4; i++)
	home_leg_pos[i][2] = float(HOME_HEIGHT);
    vec4<float> next_dir = get_move_commandf();
    
    vec4<float> step_pair[2] = { {0,0,0,0}, {0,0,0,0} };
    vec4<float> push_pair[2] = { {0,0,0,0}, {0,0,0,0} };
    
    vec4<float> step_pair_prev_pos[2], push_pair_prev_pos[2];
    step_pair_prev_pos[0] = step_pair[0];
    step_pair_prev_pos[1] = step_pair[1];
    push_pair_prev_pos[0] = step_pair[0];
    push_pair_prev_pos[1] = step_pair[1];
    // assigning the push or step role to the legs and computing the steps
    get_step(step_pair, push_pair, next_dir);
    for (int i = 0; i < 2; i++) {
	for (int j = 0; j < 3; j++) {
	    step_pair_prev_pos[i][j] = glob_leg_pos[int(step_pair[i][3])][j];
	    push_pair_prev_pos[i][j] = glob_leg_pos[int(push_pair[i][3])][j];
	}
    }
    // execute motion
    // i must be shifted by 1, so that the legs will land on the desiered position,
    // rather than counting the current position as the first of MOTION_RES steps.
    for (int i = 1; i < MOTION_RES + 1; i++) {
	add_globLeg_pushPair(push_pair, push_pair_prev_pos, next_dir, i);
	if(i * STEP_TIME_REDUCTION <= MOTION_RES + 1)
	    add_globLeg_stepPair(step_pair, step_pair_prev_pos, next_dir, i * STEP_TIME_REDUCTION);
	else if((i - 1) * STEP_TIME_REDUCTION <= MOTION_RES + 1) {
	    for (int j = 0; j < 2; j++)
		for (int h = 0; h < 3; h++)
		    step_pair_prev_pos[j][h] = glob_leg_pos[int(step_pair[j][3])][h];
	    add_globLeg_pushPair(step_pair, step_pair_prev_pos, next_dir, (i - MOTION_RES / STEP_TIME_REDUCTION)/* * float(STEP_TIME_REDUCTION)*/);
	} else 
	    add_globLeg_pushPair(step_pair, step_pair_prev_pos, next_dir, (i - MOTION_RES / STEP_TIME_REDUCTION)/* * float(STEP_TIME_REDUCTION)*/);
	delay(((i <= MOTION_RES / 2) ? i : i - (i - MOTION_RES / 2) * 2) * STEP_ACCELERATION_WEIGHT);
	update_servos();
    }
    prev_dir[1] = prev_dir[0];
    prev_dir[0] = next_dir;
}
// SIMPLE MOTION ALGORITHM END!

// Use this function for the leg assembly. For the assembly follow these steps:
// 1) Activate the tune_servo_positions() function in the loop() function.
// 2) Comment out the simple_motion_func() function.
// 3) Make sure your servos are attached (but not the carbon fiber parts (including servo discs)).
// 4) If everything is wired properly plug in the power. The servos should now jump to a specified position.
// 5) Put on the carbon fiber parts with the already attached servo discs, so that: upper leg => horizontal (pointing backwards) | servo horn (not lower leg) => vertical (pointing up) | hip assembly => horizontal (pointing right/left)
//    (if unclear email me, and I will send you a picture)
void tune_servo_positions()
{
    // leg0
    pca9685.setPWM(SERVO_CONNECTIONS[3], 0, map(servo_mask(90, 3), 0, 180, SERVOMIN, SERVOMAX));
    pca9685.setPWM(SERVO_CONNECTIONS[4], 0, map(servo_mask(135, 4), 0, 180, SERVOMIN, SERVOMAX));
    pca9685.setPWM(SERVO_CONNECTIONS[5], 0, map(servo_mask(45, 5), 0, 180, SERVOMIN, SERVOMAX));
    // leg 1
    pca9685.setPWM(SERVO_CONNECTIONS[0], 0, map(servo_mask(90, 0), 0, 180, SERVOMIN, SERVOMAX));
    pca9685.setPWM(SERVO_CONNECTIONS[1], 0, map(servo_mask(135, 1), 0, 180, SERVOMIN, SERVOMAX));
    pca9685.setPWM(SERVO_CONNECTIONS[2], 0, map(servo_mask(45, 2), 0, 180, SERVOMIN, SERVOMAX));
    // leg 2
    pca9685.setPWM(SERVO_CONNECTIONS[9], 0, map(servo_mask(90, 9), 0, 180, SERVOMIN, SERVOMAX));
    pca9685.setPWM(SERVO_CONNECTIONS[10], 0, map(servo_mask(135, 10), 0, 180, SERVOMIN, SERVOMAX));
    pca9685.setPWM(SERVO_CONNECTIONS[11], 0, map(servo_mask(45, 11), 0, 180, SERVOMIN, SERVOMAX));
    // leg 3
    pca9685.setPWM(SERVO_CONNECTIONS[6], 0, map(servo_mask(90, 6), 0, 180, SERVOMIN, SERVOMAX));
    pca9685.setPWM(SERVO_CONNECTIONS[7], 0, map(servo_mask(135, 7), 0, 180, SERVOMIN, SERVOMAX));
    pca9685.setPWM(SERVO_CONNECTIONS[8], 0, map(servo_mask(45, 8), 0, 180, SERVOMIN, SERVOMAX));
}

// reads the ps3 controller input
void get_ps3_input()
{
    x_direction = mapf(Ps3.data.analog.stick.ly, -127, 128, MAX_X_DIR, -MAX_X_DIR);
    float MAXy = sqrt(sqr(MAX_DIR_VEC_LENGTH) - sqr(x_direction));
    MAXy = MAXy >= MAX_Y_DIR ? MAX_Y_DIR : MAXy;
    y_direction = mapf(Ps3.data.analog.stick.lx, -127, 128, MAXy, -MAXy);
    rotation_direction = mapf(Ps3.data.analog.stick.rx, -127, 128, 25, -25);
    HOME_HEIGHT = mapf(Ps3.data.analog.stick.ry, -127, 128, 140, 210);

    if( Ps3.event.button_up.cross )
	new_STEP_HEIGHT = -0.2;
    if( Ps3.event.button_up.circle )
	new_STEP_HEIGHT = -0.3;
    if( Ps3.event.button_up.triangle )
	new_STEP_HEIGHT = -0.4;
    if( Ps3.event.button_up.square )
	new_STEP_HEIGHT = -0.5;

    if( Ps3.event.button_up.down )
	new_MOTION_RES = 16;
    if( Ps3.event.button_up.right )
	new_MOTION_RES = 18;
    if( Ps3.event.button_up.up )
	new_PUSH_HEIGHT = 0.1;
    if( Ps3.event.button_up.left )
	new_PUSH_HEIGHT = 0.0;
}

#if !ARDUINO_FREE
void setup(){
    // Serial monitor setup
    Serial.begin(115200);
  
    Ps3.attach(get_ps3_input);
    // You will need to put the adress of your cotroller here. A tutorial on how to get the adress:
    // https://www.youtube.com/watch?v=YhFfrJv0UDo&t=376s
    Ps3.begin("00:32:82:32:06:67");

    pca9685.begin();
    pca9685.setPWMFreq(50); // experimenting reqiered
}
#endif // !ARDUINO_FREE

void loop()
{
    // tune_servo_positions();
    simple_motion_func();
}

#if ARDUINO_FREE

void zero_Ps3()
{
    Ps3.data.analog.stick.lx = 0;
    Ps3.data.analog.stick.ly = 0;
    Ps3.data.analog.stick.rx = 0;
    Ps3.data.analog.stick.ry = 0;
    Ps3.event.button_up.cross = false;
    Ps3.event.button_up.circle = false;
    Ps3.event.button_up.triangle = false;
    Ps3.event.button_up.square = false;
    Ps3.event.button_up.down = false;
    Ps3.event.button_up.right = false;
    Ps3.event.button_up.up = false;
    Ps3.event.button_up.left = false;
}


void record_test(std::string (*test)(void), std::string test_record_name)
{
    std::ofstream record(test_record_name, std::ios::binary);
    record << test();
    std::cout << "recorded test '" << test_record_name << "'\n";
}

enum test_result_enum {
    TR_Success,
    TR_Fail,
    TR_Problem,
};

std::string get_cstr_diff(const char* a, const char* b) {
    std::string result;
    while(*(++a) && *(++b)) {
	if(*a != *b) {
	    result += *b;
	}
    }
    return result;
}

test_result_enum run_test(std::string(*test)(void), std::string test_result_name) {

    // read previous test result:
    
    std::ifstream record(test_result_name, std::ios::binary);

    char* record_data = nullptr;
    if(!record.is_open()) {
	std::cout << "Couldn't find record of test '" << test_result_name << "'.\n";
	return TR_Problem;
    }
    record.seekg(0, record.end);
    size_t record_size = record.tellg();
    record.seekg(0, record.beg);

    record_data = new char[record_size + 1];
    record.read(record_data, record_size);

    if(!record) {
	std::cout << "Error reading record: " << test_result_name << ".\n";
	record.close();
	return TR_Problem;
    }
    
    record_data[record_size] = '\0';
    record.close();
	
    // run the test and compare with record

    std::string result = test();
    std::string diff = get_cstr_diff(record_data, result.c_str());
    if(diff.size()) {
	std::cout << "\033[31m" << "FAIL " << test_result_name << "\033[0m" "\n"
		  << diff << "\n\n";
	delete[] record_data;
	return TR_Fail;
    }
    else {
	std::cout << "\033[32m" << "PASS " << test_result_name << "\033[0m" "\n";
	delete[] record_data;
	return TR_Success;
    }
}

std::string test_1()
{
    std::stringstream result;
    Serial.change_ostream(&result);
    zero_Ps3();
    int n_itr = 50;
    for(int i = 0; i < n_itr; ++i) {
	get_ps3_input();
	loop();
	Ps3.data.analog.stick.lx += 128.0 / double(n_itr);
	Ps3.data.analog.stick.ly += 128.0 / double(n_itr);
	Ps3.data.analog.stick.rx += 128.0 / double(n_itr);
	Ps3.data.analog.stick.ry += 128.0 / double(n_itr);
    }
    return result.str();
}

std::string test_walking_forward_backward()
{
    std::stringstream result;
    Serial.change_ostream(&result);
    zero_Ps3();
    Ps3.data.analog.stick.ly = -127.0;
    int n_itr = 50;
    for(int i = 0; i < n_itr; ++i) {
	get_ps3_input();
	loop();
	Ps3.data.analog.stick.ly += 255.0 / (n_itr - 1);
    }
    return result.str();
}

std::string test_walking_left_right()
{
    std::stringstream result;
    Serial.change_ostream(&result);
    zero_Ps3();
    Ps3.data.analog.stick.lx = -127.0;
    int n_itr = 50;
    for(int i = 0; i < n_itr; ++i) {
	get_ps3_input();
	loop();
	Ps3.data.analog.stick.lx += 255.0 / (n_itr - 1);
    }
    return result.str();
}

std::string test_turning()
{
    std::stringstream result;
    Serial.change_ostream(&result);
    zero_Ps3();
    Ps3.data.analog.stick.rx = -127.0;
    int n_itr = 50;
    for(int i = 0; i < n_itr; ++i) {
	get_ps3_input();
	loop();
	Ps3.data.analog.stick.rx += 255.0 / (n_itr - 1);
    }
    return result.str();
}

void record_all_tests() {
    record_test(test_1, "test_1.txt");
    record_test(test_walking_forward_backward, "test_walking_forward_backward.txt");
    record_test(test_walking_left_right, "test_walking_left_right.txt");
    record_test(test_turning, "test_turning.txt");
}

void record_comparisons() {
    record_test(test_1, "com_test_1.txt");
    record_test(test_walking_forward_backward, "com_test_walking_forward_backward.txt");
    record_test(test_walking_left_right, "com_test_walking_left_right.txt");
    record_test(test_turning, "com_test_turning.txt");
}

void run_all_tests() {
    run_test(test_1, "test_1.txt");
    run_test(test_walking_forward_backward, "test_walking_forward_backward.txt");
    run_test(test_walking_left_right, "test_walking_left_right.txt");
    run_test(test_turning, "test_turning.txt");
}



int main() {

    // run_all_tests();
    // record_comparisons();

#if VISUALIZE_QUADRUPED
	new_MOTION_RES = 20;
	
	while(!exit_graphical_environment){
	    get_ps3_input();
	    loop();
	}
#endif // VISUALIZE_QUADRUPED

}
#endif // ARDUINO_FREE
