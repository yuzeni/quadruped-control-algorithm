#pragma once

#include "../raylib50/raylib.h"
#include "../raylib50/rcamera.h"

struct QP_Leg {
    
    void draw(Vector3 angles);
    
    float upper_leg_length = 1;
    float lower_leg_length = 1;
    
    Vector3 joint_a_offset = {0,0,0};
    Vector3 joint_b_offset = {0,0,0};

    Vector3 base_pos = {0,0,0};
    bool on_mirrored_side = false;

    static const int trace_cnt = 30;
    Vector3 c_trace[trace_cnt] = {};
};


class QP_Viewport {
    
public:
    QP_Viewport(float qp_size_x, float qp_size_y, float upper_leg_length,
		float lower_leg_length, float servo_0_axis_offset, float leg_width_offset);

    bool update_viewport(Vector3 leg0, Vector3 leg1, Vector3 leg2, Vector3 leg3);
    void get_Ps3_input(double* stick_lx, double* stick_ly, double* stick_rx, double* stick_ry);
    
private:
    static const int screenWidth = 1300;
    static const int screenHeight = 1000;
    Camera camera;
    int cameraMode;
    
    QP_Leg leg0, leg1, leg2, leg3;
};
