#include "FourWire.h"

#include "../Machine/MachineConfig.h"

#include <cmath>

namespace Kinematics {
    void FourWire::group(Configuration::HandlerBase& handler) {
        handler.item("top_left_axis", _top_left_axis);
        handler.item("top_left_anchor_x", _top_left_anchor_x);
        handler.item("top_left_anchor_y", _top_left_anchor_y);

        handler.item("top_right_axis", _top_right_axis);
        handler.item("top_right_anchor_x", _top_right_anchor_x);
        handler.item("top_right_anchor_y", _top_right_anchor_y);

        handler.item("bottom_left_axis", _bottom_left_axis);
        handler.item("bottom_left_anchor_x", _bottom_left_anchor_x);
        handler.item("bottom_left_anchor_y", _bottom_left_anchor_y);

        handler.item("bottom_right_axis", _bottom_right_axis);
        handler.item("bottom_right_anchor_x", _bottom_right_anchor_x);
        handler.item("bottom_right_anchor_y", _bottom_right_anchor_y);

        handler.item("segment_length", _segment_length);
    }

    void FourWire::init() {
        log_info("Kinematic system: " << name());

        // We assume the machine starts at cartesian (0, 0, 0).
        // The motors assume they start from (0, 0, 0).
        // So we need to derive the zero lengths to satisfy the kinematic equations.
        xy_to_lengths(0, 0, zero_top_left, zero_top_right, zero_bottom_left, zero_bottom_right);
        last_motor_segment_end[0] = zero_top_left;
        last_motor_segment_end[1] = zero_top_right;
        last_motor_segment_end[2] = zero_bottom_left;
        last_motor_segment_end[3] = zero_bottom_right;        
        auto n_axis               = config->_axes->_numberAxis;             
        for (size_t axis = Z_AXIS; axis < n_axis; axis++) {                 
            last_motor_segment_end[axis] = 0.0;
        }

        init_position();
    }

    // Initialize the machine position
    void FourWire::init_position() {
        auto n_axis = config->_axes->_numberAxis;
        for (size_t axis = 0; axis < n_axis; axis++) {
            set_motor_steps(axis, 0);  // Set to zeros
        }
    }

    bool FourWire::canHome(AxisMask axisMask) {
        log_error("This kinematic system cannot home");
        return false;
    }

    bool FourWire::transform_cartesian_to_motors(float* cartesian, float* motors) {
        log_error("FourWire::transform_cartesian_to_motors is broken");
        return true;
    }

    /*
      cartesian_to_motors() converts from cartesian coordinates to motor space.

      All linear motions pass through cartesian_to_motors() to be planned as mc_move_motors operations.

      Parameters:
        target = an n_axis array of target positions (where the move is supposed to go)
        pl_data = planner data (see the definition of this type to see what it is)
        position = an n_axis array of where the machine is starting from for this move
    */
    bool FourWire::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
        float    dx, dy, dz;     // segment distances in each cartesian axis
        uint32_t segment_count;  // number of segments the move will be broken in to.

        auto n_axis = config->_axes->_numberAxis;

        float total_cartesian_distance = vector_distance(position, target, n_axis);
        if (total_cartesian_distance == 0) {
            mc_move_motors(target, pl_data);
            return true;
        }

        float cartesian_feed_rate = pl_data->feed_rate;

        // calculate the total X,Y axis move distance
        // Z axis is the same in both coord systems, so it does not undergo conversion
        float xydist = vector_distance(target, position, 2);  // Only compute distance for both axes. X and Y
        // Segment our G1 and G0 moves based on yaml file. If we choose a small enough _segment_length we can hide the nonlinearity
        segment_count = xydist / _segment_length;
        if (segment_count < 1) {  // Make sure there is at least one segment, even if there is no movement
            // We need to do this to make sure other things like S and M codes get updated properly by
            // the planner even if there is no movement??
            segment_count = 1;
        }
        float cartesian_segment_length = total_cartesian_distance / segment_count;

        // Calc length of each cartesian segment - the same for all segments
        float cartesian_segment_components[n_axis];
        for (size_t axis = X_AXIS; axis < n_axis; axis++) {
            cartesian_segment_components[axis] = (target[axis] - position[axis]) / segment_count;
        }

        float cartesian_segment_end[n_axis];
        copyAxes(cartesian_segment_end, position);

        // Calculate desired cartesian feedrate distance ratio. Same for each seg.
        for (uint32_t segment = 1; segment <= segment_count; segment++) {
            if (sys.abort) {
                return true;
            }
            // calculate the cartesian end point of the next segment
            for (size_t axis = X_AXIS; axis < n_axis; axis++) {
                cartesian_segment_end[axis] += cartesian_segment_components[axis];
            }

            // Convert cartesian space coords to motor space
            float motor_segment_end[n_axis];
            xy_to_lengths(cartesian_segment_end[X_AXIS], cartesian_segment_end[Y_AXIS], motor_segment_end[0], motor_segment_end[1], motor_segment_end[2], motor_segment_end[3]);
            for (size_t axis = Z_AXIS; axis < n_axis; axis++) {                
                motor_segment_end[axis] = cartesian_segment_end[axis];
            }

#ifdef USE_CHECKED_KINEMATICS
            // Check the inverse computation.
            float cx, cy;
            lengths_to_xy(motor_segment_end[0], motor_segment_end[1], cx, cy);

            if (abs(cartesian_segment_end[X_AXIS] - cx) > 0.1 || abs(cartesian_segment_end[Y_AXIS] - cy) > 0.1) {
                // FIX: Produce an alarm state?
            }
#endif
            // Adjust feedrate by the ratio of the segment lengths in motor and cartesian spaces,
            // accounting for all axes
            if (!pl_data->motion.rapidMotion) {  // Rapid motions ignore feedrate. Don't convert.
                                                 // T=D/V, Tcart=Tmotor, Dcart/Vcart=Dmotor/Vmotor
                                                 // Vmotor = Dmotor*(Vcart/Dcart)
                float motor_segment_length = vector_distance(last_motor_segment_end, motor_segment_end, n_axis);
                pl_data->feed_rate         = cartesian_feed_rate * motor_segment_length / cartesian_segment_length;
            }

            // TODO: G93 pl_data->motion.inverseTime logic?? Does this even make sense for wallplotter?

            // Remember the last motor position so the length can be computed the next time
            copyAxes(last_motor_segment_end, motor_segment_end);

            // Initiate motor movement with converted feedrate and converted position
            // mc_move_motors() returns false if a jog is cancelled.
            // In that case we stop sending segments to the planner.
            
            // Please note! Adjust motor direction in .yaml file by inverting direction pin for certain motor
            // There are few places in the code where it matters, so better to adjust it in .yaml config
            float cables[n_axis];
            cables[0] = motor_segment_end[0] - zero_top_left;                             
            cables[1] = motor_segment_end[1] - zero_top_right;
            cables[2] = motor_segment_end[2] - zero_bottom_left;                          
            cables[3] = motor_segment_end[3] - zero_bottom_right;

            for (size_t axis = Z_AXIS; axis < n_axis; axis++) {             
                cables[axis] = cartesian_segment_end[axis];
            }
            if (!mc_move_motors(cables, pl_data)) {
                // TODO fixup last_left last_right?? What is position state when jog is cancelled?
                log_msg("JOG IS CANCELED");
                return false;
            }
        }
        return true;
    }

    /*
      The status command uses motors_to_cartesian() to convert
      your motor positions to cartesian X,Y,Z... coordinates.

      Convert the n_axis array of motor positions to cartesian in your code.
    */
    void FourWire::motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
        // The motors start at zero, but effectively at zero_left, so we need to correct for the computation.
        // Please note! Adjust motor direction in .yaml file by inverting direction pin for certain motor
        // There are few places in the code where it matters, so better to adjust it in .yaml config
        float absolute_x, absolute_y;
        lengths_to_xy(motors[_top_left_axis] + zero_top_left, motors[_top_right_axis] + zero_top_right, motors[_bottom_left_axis] + zero_bottom_left, motors[_bottom_right_axis] + zero_bottom_right, absolute_x, absolute_y); 

        cartesian[X_AXIS] = absolute_x;
        cartesian[Y_AXIS] = absolute_y;
        for (size_t axis = Z_AXIS; axis < n_axis; axis++) {     
            cartesian[axis] = motors[axis];
        }
        // Now we have numbers that if fed back into the system should produce the same values.
    }

    /*
    Kinematic equations

    See http://paulbourke.net/geometry/circlesphere/
    See WallPlotter.cpp 
    
    These calculations are valid for the case when all 4 belts converge at one point, regardless of X, Y

    */

    void FourWire::lengths_to_xy(float top_left_length, float top_right_length, float bottom_left_length, float bottom_right_length, float& x, float& y) {
        float distance  = _top_right_anchor_x - _top_left_anchor_x;
        float distance2 = distance * distance;

        // The lengths are the radii of the circles to intersect.
        float left_radius  = top_left_length;
        float left_radius2 = left_radius * left_radius;

        float right_radius  = top_right_length;
        float right_radius2 = right_radius * right_radius;

        // Compute a and h.
        float a  = (left_radius2 - right_radius2 + distance2) / (2 * distance);
        float a2 = a * a;
        float h  = sqrtf(left_radius2 - a2);

        // Translate to absolute coordinates.
        x = _top_left_anchor_x + a;
        y = _top_left_anchor_y - h;  // flip
    }

    void FourWire::xy_to_lengths(float x, float y, float& top_left_length, float& top_right_length, float& bottom_left_length, float& bottom_right_length) {
        // Compute the hypotenuse of each triangle.

        float top_left_dy = _top_left_anchor_y - y;
        float top_left_dx = _top_left_anchor_x - x;
        top_left_length = hypot_f(top_left_dx, top_left_dy);

        float top_right_dy = _top_right_anchor_y - y;
        float top_right_dx = _top_right_anchor_x - x;
        top_right_length = hypot_f(top_right_dx, top_right_dy);

        float bottom_left_dy = _bottom_left_anchor_y - y;
        float bottom_left_dx = _bottom_left_anchor_x - x;
        bottom_left_length = hypot_f(bottom_left_dx, bottom_left_dy);

        float bottom_right_dy = _bottom_right_anchor_y - y;
        float bottom_right_dx = _bottom_right_anchor_x - x;
        bottom_right_length = hypot_f(bottom_right_dx, bottom_right_dy);

    }

    bool FourWire::kinematics_homing(AxisMask& axisMask) {
        return false;  // kinematics does not do the homing for catesian systems
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<FourWire> registration("FourWire");
    }
}
