// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
	FourWire.h

	This is a kinematic system to move a puck suspended by four cords by adjusting the cord length.
*/

#include "Kinematics.h"

namespace Kinematics {
    class FourWire : public KinematicSystem {
    public:
        FourWire() = default;

        FourWire(const FourWire&) = delete;
        FourWire(FourWire&&)      = delete;
        FourWire& operator=(const FourWire&) = delete;
        FourWire& operator=(FourWire&&) = delete;

        // Kinematic Interface

        void init() override;
        bool canHome(AxisMask axisMask) override;
        void init_position() override;
        bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) override;
        void motors_to_cartesian(float* cartesian, float* motors, int n_axis) override;
        bool transform_cartesian_to_motors(float* cartesian, float* motors) override;
        bool kinematics_homing(AxisMask& axisMask) override;

        // Configuration handlers:
        void validate() override {}
        void group(Configuration::HandlerBase& handler) override;
        void afterParse() override {}

        // Name of the configurable. Must match the name registered in the cpp file.
        const char* name() const override { return "FourWire"; }

        ~FourWire() {}

    private:
        // aka Forward Kinematics
        void lengths_to_xy(float top_left_length, float top_right_length, float bottom_left_length, float bottom_right_length, float& x, float& y);
        // aka Inverse Kinematics
        void xy_to_lengths(float x, float y, float& top_left_length, float& top_right_length, float& bottom_left_length, float& bottom_right_length);

        // State
        float zero_top_left;   //  The top left cord offset corresponding to cartesian (0, 0).
        float zero_top_right;  //  The top right cord offset corresponding to cartesian (0, 0).
        float zero_bottom_left;   //  The bottom left cord offset corresponding to cartesian (0, 0).
        float zero_bottom_right;  //  The bottom right cord offset corresponding to cartesian (0, 0).
        float last_motor_segment_end[MAX_N_AXIS];

        // Parameters
        int   _top_left_axis     = 0;
        float _top_left_anchor_x = -100;
        float _top_left_anchor_y = 100;

        int   _top_right_axis    = 1;
        float _top_right_anchor_x = 100;
        float _top_right_anchor_y = 100;

        int   _bottom_left_axis    = 2;
        float _bottom_left_anchor_x = -100;
        float _bottom_left_anchor_y = -100;

        int   _bottom_right_axis    = 3;
        float _bottom_right_anchor_x = 100;
        float _bottom_right_anchor_y = -100;

        float _segment_length = 10;
    };
}  //  namespace Kinematics
