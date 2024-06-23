
#include "../raylib50/raylib.h"
#include "../raylib50/rcamera.h"

#include <iostream>

#include "visualize_quadruped.hpp"

#include <cmath>

#define MAX_COLUMNS 20

static Vector3 operator+(Vector3 a, Vector3 b)
{
    return Vector3{a.x + b.x, a.y + b.y, a.z + b.z};
}

static Vector3 operator-(Vector3 a, Vector3 b)
{
  return Vector3{a.x - b.x, a.y - b.y, a.z - b.z};
}

static Vector3 operator-(Vector3 a, float s)
{
  return Vector3{a.x - s, a.y - s, a.z - s};
}

static Vector3 operator*(Vector3 a, float s)
{
  return Vector3{a.x * s, a.y * s, a.z * s};
}

static Vector3 operator*(Vector3 a, Vector3 b)
{
  return Vector3{a.x * b.x, a.y * b.y, a.z * b.z};
}

Vector3 normalize(const Vector3 &v)
{
    float length = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return { v.x / length, v.y / length, v.z / length };
}

struct Line {
    Vector3 a;
    Vector3 b;
};



void rotation_matrix(Vector3 a, Vector3 offset, float angle, float R[3][3])
{
    Vector3 axis = normalize(offset - a);
    float x = axis.x, y = axis.y, z = axis.z;
    float cos_theta = std::cos(angle);
    float sin_theta = std::sin(angle);
    float one_minus_cos_theta = 1.0f - cos_theta;

    R[0][0] = cos_theta + x * x * one_minus_cos_theta;
    R[0][1] = x * y * one_minus_cos_theta - z * sin_theta;
    R[0][2] = x * z * one_minus_cos_theta + y * sin_theta;

    R[1][0] = y * x * one_minus_cos_theta + z * sin_theta;
    R[1][1] = cos_theta + y * y * one_minus_cos_theta;
    R[1][2] = y * z * one_minus_cos_theta - x * sin_theta;

    R[2][0] = z * x * one_minus_cos_theta - y * sin_theta;
    R[2][1] = z * y * one_minus_cos_theta + x * sin_theta;
    R[2][2] = cos_theta + z * z * one_minus_cos_theta;
}

static Vector3 rotate_point_around_line(Vector3 p, Vector3 offset, float R[3][3])
{
    // Step 1: Translate so that 'offset' is at the origin
    Vector3 pt = p - offset;
    
    // Step 4: Rotate the point
    Vector3 rotatedPt = {
        pt.x * R[0][0] + pt.y * R[0][1] + pt.z * R[0][2],
        pt.x * R[1][0] + pt.y * R[1][1] + pt.z * R[1][2],
        pt.x * R[2][0] + pt.y * R[2][1] + pt.z * R[2][2]
    };
    
    return rotatedPt + offset;
}

Vector3 cross(Vector3 v1, Vector3 v2) {
    return {v1.y * v2.z - v1.z * v2.y,
	    v1.z * v2.x - v1.x * v2.z,
	    v1.x * v2.y - v1.y * v2.x};
}

void QP_Leg::draw(Vector3 angles)
{

    if(on_mirrored_side)
	angles.x = 180 - angles.x;

    angles = (angles - Vector3{90, 135, 45}) * DEG2RAD;
    	
    Vector3 a = Vector3{0, 0, 0};
    Vector3 a_o = a + joint_a_offset;
    Vector3 b = a_o + Vector3{-upper_leg_length, 0, 0};
    Vector3 b_o = b + joint_b_offset;
    Vector3 c = b_o + Vector3{0, -lower_leg_length, 0};

    float R[3][3];
    Vector3 offset;
	
    // rotate b_o and c (lower leg) around b
    offset = b + cross(b - a_o, c - b_o);
    rotation_matrix(b, offset, angles.z, R);
    b_o = rotate_point_around_line(b_o, offset, R);
    c = rotate_point_around_line(c, offset, R);
	
    // rotate b and b_o around a_o and move c by the difference of old b_o to new b_o
    Vector3 old_b_o = b_o;
    offset = a_o + cross(a_o - b, c - b_o);
    rotation_matrix(a_o, offset, angles.y, R);
    b = rotate_point_around_line(b, offset, R);
    b_o = rotate_point_around_line(b_o, offset, R);
    c = c + (b_o - old_b_o);

    // rotate a_o, b, b_o and c around a
    offset = a + Vector3{1,0,0};
    rotation_matrix(a, offset, angles.x, R);
    a_o = rotate_point_around_line(a_o, offset, R);
    b = rotate_point_around_line(b, offset, R);
    b_o = rotate_point_around_line(b_o, offset, R);
    c = rotate_point_around_line(c, offset, R);
	
    a = a + base_pos;
    a_o = a_o + base_pos;
    b = b + base_pos;
    b_o = b_o + base_pos;
    c = c + base_pos;

    DrawLine3D(a, a_o, BLACK);
    DrawLine3D(a_o, b, RED);
    DrawLine3D(b_o, c, BLUE);

    for(int i = trace_cnt - 1; i >= 1; --i)
	c_trace[i] = c_trace[i - 1];
    c_trace[0] = c;
	
    for(int i = 0; i < trace_cnt - 1; ++i)
	DrawLine3D(c_trace[i], c_trace[i + 1], GREEN);
}

bool QP_Viewport::update_viewport(Vector3 angles_leg0, Vector3 angles_leg1, Vector3 angles_leg2, Vector3 angles_leg3)
{
    if(WindowShouldClose()) {
	CloseWindow();
	return false;
    }
    
    UpdateCamera(&camera, cameraMode);

    BeginDrawing();

    ClearBackground(RAYWHITE);

    BeginMode3D(camera);

    DrawGrid(20, 1);

    // static float angle = 0;
    // angle += 0.1;
    // leg0.draw({90,135+angle,45+angle});
    // leg1.draw({90,135+angle,45+angle});
    // leg2.draw({90,135+angle,45+angle});
    // leg3.draw({90,135+angle,45+angle});

    // leg0.draw({0,0,0});
    // leg1.draw({0,0,0});
    // leg2.draw({0,0,0});
    // leg3.draw({0,0,0});

    leg0.draw(angles_leg0);
    leg1.draw(angles_leg1);
    leg2.draw(angles_leg2);
    leg3.draw(angles_leg3);
    
    EndMode3D();

    EndDrawing();

    return true;
}

void QP_Viewport::get_gamepad_input(double* stick_lx, double* stick_ly, double* stick_rx, double* stick_ry) {
    if (IsGamepadAvailable(0))
    {
	*stick_lx = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X) * 100.0;
	*stick_ly = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y) * 100.0;
	*stick_rx = GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_X) * 100.0;
	*stick_ry = GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_Y) * 100.0;
    }
};
