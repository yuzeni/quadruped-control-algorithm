#pragma once

#include "../raylib50/raylib.h"
#include "../raylib50/rcamera.h"

struct QP_Leg {
    
    void draw(Vector3 angles);
    
    float upper_leg_length = 1.3;
    float lower_leg_length = 1.3;
    
    Vector3 joint_a_offset = {0,0,0};
    Vector3 joint_b_offset = {0,0,0};

    Vector3 base_pos = {0,0,0};
    bool on_mirrored_side = false;

    static const int trace_cnt = 20;
    Vector3 c_trace[trace_cnt] = {};
};


class QP_Viewport {
    
public:
    QP_Viewport(float qp_size_x, float qp_size_y, float upper_leg_length,
		float lower_leg_length, float servo_0_axis_offset, float leg_width_offset)

    {
	InitWindow(screenWidth, screenHeight, "raylib [core] example - 3d camera first person");

	// Define the camera to look into our 3d world (position, target, up vector)
	camera.position = Vector3{ 0.0f, 2.0f, 4.0f };    // Camera position
	camera.target = Vector3{ 0.0f, 2.0f, 0.0f };      // Camera looking at point
	camera.up = Vector3{ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
	camera.fovy = 60.0f;                                // Camera field-of-view Y
	camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

	cameraMode = CAMERA_THIRD_PERSON;

	DisableCursor();                    // Limit cursor to relative movement inside the window
	SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second

	qp_size_x /= 100;
	qp_size_y /= 100;
	upper_leg_length /= 100;
	lower_leg_length /= 100;
	servo_0_axis_offset /= 100;
	leg_width_offset /= 100;

	leg0.joint_a_offset = {0, 0, servo_0_axis_offset};
	leg0.joint_b_offset = {0, 0, leg_width_offset};
	leg0.base_pos = {qp_size_x / 2, 2, qp_size_y / 2};
	leg0.lower_leg_length = lower_leg_length;
	leg0.upper_leg_length = upper_leg_length;
	leg0.on_mirrored_side = true;

	leg1.joint_a_offset = {0, 0, -servo_0_axis_offset};
	leg1.joint_b_offset = {0, 0, -leg_width_offset};
	leg1.base_pos = {qp_size_x / 2, 2, -qp_size_y / 2};
	leg1.lower_leg_length = lower_leg_length;
	leg1.upper_leg_length = upper_leg_length;

	leg2.joint_a_offset = {0, 0, servo_0_axis_offset};
	leg2.joint_b_offset = {0, 0, leg_width_offset};
	leg2.base_pos = {-qp_size_x / 2, 2, qp_size_y / 2};
	leg2.lower_leg_length = lower_leg_length;
	leg2.upper_leg_length = upper_leg_length;
	leg2.on_mirrored_side = true;

	leg3.joint_a_offset = {0, 0, -servo_0_axis_offset};
	leg3.joint_b_offset = {0, 0, -leg_width_offset};
	leg3.base_pos = {-qp_size_x / 2, 2, -qp_size_y / 2};
	leg3.lower_leg_length = lower_leg_length;
	leg3.upper_leg_length = upper_leg_length;
    }

    bool update_viewport(Vector3 leg0, Vector3 leg1, Vector3 leg2, Vector3 leg3);

    void get_gamepad_input(double* stick_lx, double* stick_ly, double* stick_rx, double* stick_ry);
    
private:
    
    static const int screenWidth = 1300;
    static const int screenHeight = 1000;
    Camera camera;
    int cameraMode;
    
    QP_Leg leg0, leg1, leg2, leg3;
};
