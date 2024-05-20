#include "mbed.h"

// pes board pin map
#include "pm2_drivers/PESBoardPinMap.h"

// drivers
#include "pm2_drivers/DebounceIn.h"

// Eigen Lib
#include "eigen/Dense.h"

// DC motor lib
#include "pm2_drivers/DCMotor.h"

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(USER_BUTTON); // create DebounceIn object to evaluate the user button
                                     // falling and rising edge
void toggle_do_execute_main_fcn();   // custom function which is getting executed when user
                                     // button gets pressed, definition below

// main runs as an own thread
int main()
{
    // attach button fall function address to user button object, button has a pull-up resistor
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, there for
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(USER_LED);

    // DC Motot Intit
    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                     // 6.0f V if you only use one battery pack
    const float gear_ratio = 100.00f; 
    const float kn = 140.0f / 12.0f;
    // motor M1 and M2, do NOT enable motion planner, disabled per default
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    // additional dc motor settings (don't use them for the line follower)
    motor_M1.enableMotionPlanner();
    motor_M1.setMaxVelocity(motor_M1.getMaxVelocity() * 1.0f);
    motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 1.0f);
    motor_M1.setVelocityCntrlIntegratorLimitsPercent(100.0f);
    motor_M2.enableMotionPlanner();
    motor_M2.setMaxVelocity(motor_M2.getMaxVelocity() * 1.0f);
    motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 1.0f);
    motor_M2.setVelocityCntrlIntegratorLimitsPercent(100.0f);

    // // robot kinematics
    // const float r_wheel = 0.034f / 2.0f; // wheel radius in meters
    // const float b_wheel = 0.13f;          // wheelbase, distance from wheel to wheel in meters
    // Eigen::Matrix2f Cwheel2robot; // transform wheel to robot
    // Cwheel2robot <<  r_wheel / 2.0f   ,  r_wheel / 2.0f   ,
    //                 r_wheel / b_wheel, -r_wheel / b_wheel;
    // Eigen::Vector2f robot_coord = {0.0f, 0.0f};  // contains v and w (robot translational and rotational velocity)
    // Eigen::Vector2f wheel_speed = {0.0f, 0.0f};  // w1 w2 (wheel speed)

    // const float L_square = 1.4f;     // forward distance in meters
    // const float turn = M_PI / 2.0f; // rotation angle in radians
    //                                  // -90 deg for right turn (CW), +90 deg for left turn (CCW)

    // // calculate pure forward and pure turn movement as wheel angles
    // Eigen::Vector2f robot_coord_forward = {L_square, 0.0f};
    // Eigen::Vector2f robot_coord_turn = {0.0, turn};
    // Eigen::Vector2f wheel_angle_forward = Cwheel2robot.inverse() * robot_coord_forward;
    // Eigen::Vector2f wheel_angle_turn = Cwheel2robot.inverse() * robot_coord_turn;

    // set up states for state machine
    enum RobotState {
        FORWARD,
        TURN,
        RESET
    } robot_state = RobotState::FORWARD;

    const float angle_threshold = 0.005f;
    int turn_cntr = 0;
 
    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {
            enable_motors = 1;
            // state machine
            switch (robot_state) {
                case RobotState::FORWARD:
                    motor_M1.setRotationRelative(wheel_angle_forward(0) / (2.0f * M_PI));
                    motor_M2.setRotationRelative(wheel_angle_forward(1) / (2.0f * M_PI));
                    robot_state = RobotState::TURN;
                    break;
                case RobotState::TURN:
                    if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                        (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                        motor_M1.setRotationRelative(wheel_angle_turn(0) / (2.0f * M_PI));
                        motor_M2.setRotationRelative(wheel_angle_turn(1) / (2.0f * M_PI));
                        robot_state = RobotState::FORWARD_OR_RESET;
                    }
                    break;
                case RobotState::FORWARD_OR_RESET:
                    if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                        (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                        turn_cntr++;
                        if (turn_cntr == 4) {
                            robot_state = RobotState::RESET;
                        } else {
                            robot_state = RobotState::FORWARD;
                        }
                    }
                    break;
                case RobotState::RESET:
                    toggle_do_execute_main_fcn();
                    turn_cntr = 0;
                    robot_state = RobotState::FORWARD;
                    break;
                default:
                    break;
            } 
            // debugging
            printf("RS: %d, M1 SP: %.3f, M2 SP: %.3f, M1: %.3f, M2: %.3f, TC: %d\n", robot_state
                                                                                   , motor_M1.getRotationTarget()
                                                                                   , motor_M2.getRotationTarget()
                                                                                   , motor_M1.getRotation()
                                                                                   , motor_M2.getRotation()
                                                                                   , turn_cntr);
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;
            }
        }
        
        // toggling the user led
        user_led = !user_led;
      
        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}