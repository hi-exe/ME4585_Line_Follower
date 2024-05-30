#include "mbed.h"

// pes board pin map
#include "pm2_drivers/PESBoardPinMap.h"

// drivers
#include "pm2_drivers/DebounceIn.h"

// Eigen Lib
#include "eigen/Dense.h"

// DC motor lib
#include "pm2_drivers/DCMotor.h"

// Line Sensor lib
#include "pm2_drivers/SensorBar.h"

// Line Follower lib
#include "pm2_drivers/LineFollower.h"

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
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max); //Right Motor
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max); //Left Motor

    // Line Follower Init
    const float d_wheel = 0.053f;  // wheel diameter in meters
    const float b_wheel = 0.153f; // wheelbase, distance from wheel to wheel in meters
    const float bar_dist = 0.132f; // distance from wheel axis to leds on sensor bar / array in meters
    // line follower
    LineFollower lineFollower(PB_9, PB_8, bar_dist, d_wheel, b_wheel, motor_M2.getMaxPhysicalVelocity());

    // // Parametres Adjustment
    // const float Kp{5.0f}; //Proportional Gain
    // const float Kp_nl{5.0f}; //Non-linear Gain
    // void setRotationalVelocityGain(float Kp, float Kp_nl)

    // // velocity controller data
    // const float wheel_vel_max = 2.0f * M_PI * motor_M2.getMaxPhysicalVelocity();
    // void setMaxWheelVelocityRPS(float wheel_vel_max) 
    // //This parameter limits the maximum wheel velocity (argument in rotations per second), indirectly affecting the robot's linear and angular velocities.
    // //The user can adjust this limit to tune the performance of their system.

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {
           
            // visual feedback that the main task is executed, setting this once would actually be enough
            enable_motors = 1;
            motor_M1.setVelocity(lineFollower.getRightWheelVelocity()); // set a desired speed for speed controlled dc motors M1
            motor_M2.setVelocity(lineFollower.getLeftWheelVelocity());  // set a desired speed for speed controlled dc motors M2
            
            // // debugging
            // printf("RS: %d, M1 SP: %.3f, M2 SP: %.3f, M1: %.3f, M2: %.3f, TC: %d\n", robot_state
            //                                                                        , motor_M1.getRotationTarget()
            //                                                                        , motor_M2.getRotationTarget()
            //                                                                        , motor_M1.getRotation()
            //                                                                        , motor_M2.getRotation()
            //                                                                        , turn_cntr);
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset variables and objects

                enable_motors = 0;
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