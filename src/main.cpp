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

#include <vector>
#include <cstdint>
#include <cmath>

#if !ARDUINO_FREE
#  include <Ps3Controller.h>
#  include <Adafruit_PWMServoDriver.h>
#else
#include <iostream>
#include <chrono>
#include <thread>

/* arduino specific functionality */

// source: https://www.arduino.cc/reference/en/language/functions/math/map/
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

struct {
    template<typename T> void print(T arg) { std::cout << arg; }
    template<typename T> void println(T arg) { std::cout << arg << '\n'; }
    void println() { std::cout << '\n'; }
    
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

// SERVO CONSTANTS

// Define maximum and minimum number of "ticks" for the servo motors
// Range from 0 to 4095
// This determines the pulse width
#define SERVOMIN  100// experimenting reqiered.
#define SERVOMAX  550
 
// Define servo motor connections
const int SERVO_CONNECTIONS[12] = {11, 10, 9, 15, 14, 13, 4, 5, 6, 0, 1, 2};

// SIMPLE MOTION ALORITHM CONSTANTS START

#define INF 99999999999

// you might not need this threshold, experimenting required
#define CONTROLLER_DRIFT_THRESHOLD 4

#define NUMBER_OF_LEGS 4
#define SERVOS_PER_LEG 3

// The QP_SIZE can be set to anything. For example Increasing the Y SIZE, increases the distance
// between the left and right legs. Increasing X SIZE will increase the distance between the front and rear legs.
// ACTUAL_QP_SIZE is needed for the computation, and should represent the exact distances
// between the hip joints (Y) and the pivots of the upper legs (X)
#define QP_SIZEX 210
#define ACTUAL_QP_SIZEX 218
#define QP_SIZEY 210
#define ACTUAL_QP_SIZEY 110


#define UPLEGLGTH 130  // UPLEGLGTH is the length of the upper leg in mm
#define LOWLEGLGTH 130 // LOWLEGLGTH is the length of the lower leg in mm

#define LEG_OFFSET_Y 32.4  // This is the Y distance between the hip joint and the leg tip
#define LOWLEG_CURVATURE 7 // Takes the curvature of the lower leg into account, in degrees

// Center of gravity offset in mm from the midpoint of the quadruped
#define COG_X_OFFSET -10
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

// mathematical

constexpr double EPSILON = 0.000000001;
constexpr double qp_PI = 3.14159265359;
constexpr double qp_RAD_TO_DEG =  180.0 / qp_PI;
constexpr double qp_DEG_TO_RAD =  qp_PI / 180.0;

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
vector<vector<float>> home_leg_pos = {
    { float(QP_SIZEX / 2 + COG_X_OFFSET), float(-ACTUAL_QP_SIZEY / 2 + COG_Y_OFFSET), float(HOME_HEIGHT) },
    { float(-QP_SIZEX / 2 + COG_X_OFFSET), float(-ACTUAL_QP_SIZEY / 2 + COG_Y_OFFSET), float(HOME_HEIGHT) },
    { float(QP_SIZEX / 2 + COG_X_OFFSET), float(ACTUAL_QP_SIZEY / 2 + COG_Y_OFFSET), float(HOME_HEIGHT) },
    { float(-QP_SIZEX / 2 + COG_X_OFFSET), float(ACTUAL_QP_SIZEY / 2 + COG_Y_OFFSET), float(HOME_HEIGHT) },
};
// Also needed for the "Actual" quadruped size.
const vector<vector<float>> actual_home_leg_pos = {
    { float(ACTUAL_QP_SIZEX / 2), float(-QP_SIZEY / 2), float(HOME_HEIGHT) },
    { float(-ACTUAL_QP_SIZEX / 2), float(-QP_SIZEY / 2), float(HOME_HEIGHT) },
    { float(ACTUAL_QP_SIZEX / 2), float(QP_SIZEY / 2), float(HOME_HEIGHT) },
    { float(-ACTUAL_QP_SIZEX / 2), float(QP_SIZEY / 2), float(HOME_HEIGHT) },
};

// This is where the current leg position is stored. (very important variable)
vector<vector<float>> glob_leg_pos = home_leg_pos;

float get_angle(float x, float y);

// these are the angles of the vectors to the leg tips in their "home" position
vector<float> home_leg_angle = {
    get_angle(home_leg_pos[0][0], home_leg_pos[0][1]),
    get_angle(home_leg_pos[1][0], home_leg_pos[1][1]),
    get_angle(home_leg_pos[2][0], home_leg_pos[2][1]),
    get_angle(home_leg_pos[3][0], home_leg_pos[3][1]),
};

// in case the IK algorithm cant find a solution, the previous angles will be used
vector<vector<float>> previous_angles = {
    { 90, 90, 90 },
    { 90, 90, 90 },
    { 90, 90, 90 },
    { 90, 90, 90 }
};

// The previous step pair. 
int prev_step_pair = int(!START_PAIR);

// Here the two last directions (global direction of the quadruped) are saved
vector<vector<float>> prev_dir{ { 0,0,0,0 }, {0,0,0,0} };

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
double angle_vec2_vec2(vector<double> a, vector<double> b) {
    return (acos((a[0] * b[0] + a[1] * b[1]) / (sqrt(sqr(a[0]) + sqr(a[1])) * sqrt(sqr(b[0]) + sqr(b[1]))))) * double(qp_RAD_TO_DEG);
}

// Inverse Kinematics Algorithm, might be buggy
// pos = (x,y,z) in regards to the local coordinate system of the leg
// leg = id of the leg
vector<float> get_IK(vector<float> pos, int leg)
{
    pos[0] = float(pos[0]);
    pos[1] = float(pos[1]);
    pos[2] = float(pos[2]);
    vector<double> knee(2);
    vector<float> angles;
    vector<double> trans_pos(3);
    double l1 = sqrt(sqr(pos[1]) + sqr(pos[2]));
    double C = (qp_PI / 2 - asin(LEG_OFFSET_Y / l1)) * qp_RAD_TO_DEG;
    double Cdif = 90 - C;
    if (leg >= 2)
	Cdif *= -1;
    double l3 = l1 * sin(C * qp_DEG_TO_RAD);
    trans_pos[0] = pos[0];
    trans_pos[1] = (l3 / sqrt(sqr(pos[1]) + sqr(pos[2]))) * pos[1];
    trans_pos[2] = (l3 / sqrt(sqr(pos[1]) + sqr(pos[2]))) * pos[2];
    double test = sqrt(sqr(trans_pos[1]) + sqr(trans_pos[2]));
    trans_pos[1] = trans_pos[1] * cos(Cdif * qp_DEG_TO_RAD) - trans_pos[2] * sin(Cdif * qp_DEG_TO_RAD);
    trans_pos[2] = trans_pos[1] * sin(Cdif * qp_DEG_TO_RAD) + trans_pos[2] * cos(Cdif * qp_DEG_TO_RAD);
    double test1 = sqrt(sqr(trans_pos[1]) + sqr(trans_pos[2]));
    trans_pos[1] = (l3 / sqrt(sqr(trans_pos[1]) + sqr(trans_pos[2]))) * trans_pos[1];
    trans_pos[2] = (l3 / sqrt(sqr(trans_pos[1]) + sqr(trans_pos[2]))) * trans_pos[2];
    //trans_pos[1] = -trans_pos[1]; // the trans_pos[1] also needs to be flipt, since trans_pos[2] is already flipped
    double test2 = sqrt(sqr(trans_pos[1]) + sqr(trans_pos[2]));
    // extending the length of the [0][2] vector to the length of the [0][1][2] vector
    double q1 = sqrt((sqr(trans_pos[0]) + sqr(trans_pos[1]) + sqr(trans_pos[2])) / (sqr(trans_pos[0]) + sqr(trans_pos[2]) + EPSILON));
    trans_pos[0] *= q1;
    trans_pos[2] *= q1;

    // Calculation of the intersection points of the circles, created by the radii of the legs (Source: http://paulbourke.net/geometry/circlesphere/tvoght.c)
    double x0 = 0, y0 = 0, x1 = trans_pos[0], y1 = trans_pos[2]; // mid points of the two circles
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
    knee[0] = fx;
    knee[1] = fy;
    // calculating the angles between the computed vectors and the vectors representing the minimal servo position
    angles = {
	float(angle_vec2_vec2({ (leg >= 2 ? 1.0 : -1.0), 0 }, { trans_pos[1], trans_pos[2] })), // 0
	float(angle_vec2_vec2({1.0, 1.0}, knee)), // 1
	float(angle_vec2_vec2({ knee[0] - trans_pos[0], knee[1] - trans_pos[2] }, { 1,-1 })) // 2
    };
    // in case one of the angles is not a number (meaning there isnt a solution) then it returns the previous angles 
    if (angles[0] + angles[1] + angles[2] != angles[0] + angles[1] + angles[2])
	return previous_angles[leg];
    previous_angles[leg] = angles;
    return angles;
}

// servo_mask() Is used to correct for slightly incorrect leg assembly (wrong leg segment orientation) AND to take into account,
// that some servos positions have to be mirrored by subtracting the servo position from the maximum servo position (180°)
// ATTENTION!! The current values that are subtracted from the "angle" were my personal values and are there for demonstration purposes.
// You will need to change them to 0 and then after the assembly of the robot, add or subtract something in order to adjust the physical leg orientation to match the software model !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Use the tune_servo_positions() function for the assembly in order to get the orientation of them right. This function can be found close to the bottom.
float servo_mask(float angle, int servo){
    // 0
    if(servo == 3)
	return(180.0 - (angle - 22));
    else if(servo == 4)
	return(180.0 - (angle - 38));
    else if(servo == 5)
	return(180.0 - (angle - 29.5));
    // 1
    else if(servo == 0)
	return(angle - 13);
    else if(servo == 1)
	return(180.0 - (angle - 32));
    else if(servo == 2)
	return(180.0 - (angle - 31));
    // 2
    else if(servo == 9)
	return(angle - 13);
    else if(servo == 10)
	return(angle - 38);
    else if(servo == 11)
	return(angle - 28);
    // 3 
    else if(servo == 6)
	return(180.0 - (angle - 11));
    else if(servo == 7)
	return(angle - 36);
    else if(servo == 8)
	return(angle - 29);
}

// moves the servos to whatever is currently in glob_leg_pos
void update_servos()
{
    // Subtracting the "actual home leg position" from the global leg positions,
    // in order to move the coordinates from the global to the local coordinate system  
    vector<vector<float>> new_leg_pos = { {0,0,0},{0,0,0},{0,0,0},{0,0,0} };
    vector<float> new_servo_pos;
    for(int i = 0; i < 4; i++){
	for(int j = 0; j < 2; j++)
	    new_leg_pos[i][j] = glob_leg_pos[i][j] - actual_home_leg_pos[i][j];
	new_leg_pos[i][2] = glob_leg_pos[i][2];
	new_leg_pos[i][0] *= -1;
    }
  
    // Iterating over all servos and moving them to the new position
    for(int leg = 0; leg < NUMBER_OF_LEGS; leg++){
	new_servo_pos = get_IK(new_leg_pos[leg], leg);
	for(int servo = 0; servo < SERVOS_PER_LEG; servo++){
	    // setting the servo positions
	    pca9685.setPWM(SERVO_CONNECTIONS[leg * SERVOS_PER_LEG + servo], 0, map(servo_mask(new_servo_pos[servo], leg * SERVOS_PER_LEG + servo), 0, 180, SERVOMIN, SERVOMAX));
	    // printing the servo angles in the serial monitor
	    Serial.print(SERVO_CONNECTIONS[leg * SERVOS_PER_LEG + servo]);
	    Serial.print(", ");
	    Serial.print(servo_mask(new_servo_pos[servo], SERVO_CONNECTIONS[leg * SERVOS_PER_LEG + servo]));
	    Serial.print(" | ");
	}
	Serial.print("  ");
    }
    Serial.println();
}

// like the map() function, but for float instead of int
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// experimental. This will have to be aplied on "new_dir" in get_move_commandf()
void adapt_motion_params(vector<float> new_dir){
    float dir_vec_length = sqrt(sqr(new_dir[0]) + sqr(new_dir[1]) + sqr(new_dir[2]));
    // MOTION_RES should bean integer, therefore map instead of mapf is used
    MOTION_RES = map(dir_vec_length, 0, MAX_DIR_VEC_LENGTH, MIN_MOTION_RES, MAX_MOTION_RES);
    if((dir_vec_length) > (MAX_DIR_VEC_LENGTH * WALK_RUN_PIVOT)) {
	STEP_TIME_REDUCTION = 1;
	PUSH_HEIGHT = MAX_PUSH_HEIGHT;
    }
    else {
	STEP_TIME_REDUCTION = mapf(dir_vec_length, 0, MAX_DIR_VEC_LENGTH * WALK_RUN_PIVOT, MAX_STEP_TIME_REDUCTION, MIN_STEP_TIME_REDUCTION);
	PUSH_HEIGHT = 0;
    }
}

vector<float> get_move_commandf(){
    vector<float> new_dir = {(float)x_direction, (float)y_direction, (float)z_direction, (float)rotation_direction};
    return(new_dir);
}

float get_angle(float x, float y) {
    return atan2(y, x) * float(qp_RAD_TO_DEG) + ((y <= 0) * 360);
}

vector<float> vec_float_div(vector<float> a, float b) {
    vector<float> quo(a.size());
    for (int i = 0; i < a.size(); i++)
	quo[i] = (a[i] / b);
    return(quo);
}

vector<float> vec_float_mul(vector<float> a, float b) {
    vector<float> pro(a.size());
    for (int i = 0; i < a.size(); i++)
	pro[i] = (a[i] * b);
    return(pro);
}

void get_rota_trans(vector<vector<float>>& next_step, vector<float>& next_dir) {
    float r;
    for (int i = 0; i < next_step.size(); i++) {
	r = sqrt(sqr(next_step[i][0]) + sqr(next_step[i][1]));
	next_step[i][0] = cos(next_dir[3] * float(qp_DEG_TO_RAD)) * next_step[i][0] - sin(next_dir[3] * float(qp_DEG_TO_RAD)) * next_step[i][1];
	next_step[i][1] = sin(next_dir[3] * float(qp_DEG_TO_RAD)) * next_step[i][0] + cos(next_dir[3] * float(qp_DEG_TO_RAD)) * next_step[i][1];
    }
}

void get_step(vector<vector<float>>& step_pair, vector<vector<float>>& push_pair, vector<float>& next_dir) {
    vector<vector<float>> next_step = { {0,0,0,0}, {0,0,0,0} };
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
	    step_pair[i][j] = home_leg_pos[step_pair[i][3]][j] + next_dir[j] / 2;
    get_rota_trans(step_pair, next_dir);
}

vector<float> bez3_at(float* p0, float* p1, float* p2, float at, int dim) {
    float p01, p12, p01_12;
    vector<float> bez_result_at;
    for (int j = 0; j < dim; j++) {
	p01 = (p0[j] + (p1[j] - p0[j]) * at);
	p12 = (p1[j] + (p2[j] - p1[j]) * at);
	p01_12 = (p01 + (p12 - p01) * at);
	bez_result_at.push_back(p01_12);
    }
    return(bez_result_at);
}

vector<float> incl_z(vector<float>& next_leg_pos, vector<float>& prev_leg_pos, float z_peak, int i) {
    float p0[3] = { prev_leg_pos[0], prev_leg_pos[1], prev_leg_pos[2] };
    float p1[3] = { (next_leg_pos[0] + prev_leg_pos[0]) / 2, (next_leg_pos[1] + prev_leg_pos[1]) / 2, ((next_leg_pos[2] + prev_leg_pos[2]) / 2) + ((next_leg_pos[2] + prev_leg_pos[2]) / 2) * float(z_peak) };
    float p2[3] = { next_leg_pos[0], next_leg_pos[1], next_leg_pos[2] };
    return bez3_at(p0, p1, p2, (i >= 0 ? i / MOTION_RES : 0), 3);
}

void add_globLeg_stepPair(vector<vector<float>>& step_pair, vector<vector<float>>& step_pair_prev_pos, vector<float>& next_dir, int i) {
    for (int j = 0; j < step_pair.size(); j++)
	glob_leg_pos[step_pair[j][3]] = incl_z(step_pair[j], step_pair_prev_pos[j], (sqrt(sqr(next_dir[0] + prev_dir[0][0] + prev_dir[1][0]) + sqr(next_dir[1] + prev_dir[0][1] + prev_dir[1][1]) + sqr(next_dir[2] + prev_dir[0][2] + prev_dir[1][2]) + sqr(next_dir[3] + prev_dir[0][3] + prev_dir[1][3])) >= 4 ? STEP_HEIGHT : 0), i);
}

void add_globLeg_pushPair(vector<vector<float>>& push_pair, vector<vector<float>>& push_pair_prev_pos, vector<float>& next_dir, int i) {
    vector<float> next_dir_portions = vec_float_div(next_dir, MOTION_RES), next_dir_portions_x_i;
    for (int j = 0; j < push_pair.size(); j++) {
	// assigning the targeted end position for the pushing legs
	for(int h = 0; h < 3; h++)
	    push_pair[j][h] = push_pair_prev_pos[j][h] - next_dir[h];
	// including the rotational motion
	glob_leg_pos[push_pair[j][3]] = incl_z(push_pair[j], push_pair_prev_pos[j], (sqrt(sqr(next_dir[0] + prev_dir[0][0] + prev_dir[1][0]) + sqr(next_dir[1] + prev_dir[0][1] + prev_dir[1][1]) + sqr(next_dir[2] + prev_dir[0][2] + prev_dir[1][2]) + sqr(next_dir[3] + prev_dir[0][3] + prev_dir[1][3])) >= 4 ? PUSH_HEIGHT : 0), i);
	next_dir_portions_x_i = vec_float_mul(next_dir_portions, -i); // we need a minus, since the pushing pair has to go in the reversed direction of the global direction
    }
    vector<vector<float>> next_step{ glob_leg_pos[push_pair[0][3]],glob_leg_pos[push_pair[1][3]] };
    get_rota_trans(next_step, next_dir_portions_x_i);
    for (int j = 0; j < next_step.size(); j++)
	for (int h = 0; h < 2; h++)
	    glob_leg_pos[push_pair[j][3]][h] = next_step[j][h];
}

void simple_motion_func() {
    PUSH_HEIGHT = new_PUSH_HEIGHT;
    STEP_HEIGHT = new_STEP_HEIGHT;
    MOTION_RES = new_MOTION_RES;
    for (int i = 0; i < 4; i++)
	home_leg_pos[i][2] = float(HOME_HEIGHT);
    vector<float> next_dir = get_move_commandf();
    vector<vector<float>> step_pair = { {0,0,0,0}, {0,0,0,0} }, push_pair = { {0,0,0,0}, {0,0,0,0} };
    vector<vector<float>> step_pair_prev_pos, push_pair_prev_pos;
    step_pair_prev_pos = step_pair;
    push_pair_prev_pos = step_pair;
    // assigning the push or step role to the legs and computing the steps
    get_step(step_pair, push_pair, next_dir);
    for (int i = 0; i < 2; i++) {
	for (int j = 0; j < 3; j++) {
	    step_pair_prev_pos[i][j] = glob_leg_pos[step_pair[i][3]][j];
	    push_pair_prev_pos[i][j] = glob_leg_pos[push_pair[i][3]][j];
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
		    step_pair_prev_pos[j][h] = glob_leg_pos[step_pair[j][3]][h];
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
void tune_servo_positions(){
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
void get_ps3_input(){
    x_direction = mapf(Ps3.data.analog.stick.ly, -127, 128, -MAX_X_DIR, MAX_X_DIR);
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
    //tune_servo_positions();
    simple_motion_func();
}

#if ARDUINO_FREE
int main() {

    for(int i = 0; i < 1; ++i) {
	std::cout << "\033[31m" "loop itr: " <<  i << "\033[0m" "\n";
	loop();
    }
    
}
#endif // ARDUINO_FREE
