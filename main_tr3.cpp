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



    // Sensor Bar Init
    // robot kinematics
    const float r1_wheel = 0.035f/2.0f; // right wheel radius in meters
    const float r2_wheel = 0.035f/2.0f; // left  wheel radius in meters
    const float b_wheel = 0.1518f; // wheelbase, distance from wheel to wheel in meters
    Eigen::Matrix2f Cwheel2robot; // transform wheel to robot
    Cwheel2robot <<  r1_wheel / 2.0f   ,  r2_wheel / 2.0f   ,
                    r1_wheel / b_wheel, -r2_wheel / b_wheel;

    const float bar_dist = 0.118f; // distance from wheel axis to leds on sensor bar / array in meters
    SensorBar sensorBar(PB_9, PB_8, bar_dist);

    // angle measured from sensor bar (black line) relative to robot
    float angle{0.0f};
    const float Kp{8.0f};
    const float wheel_vel_max = 2.0f * M_PI * motor_M2.getMaxPhysicalVelocity();

    // set up states for state machine
    enum RobotState {
        INITIAL,
        LINE_FOLLOW,
        BRICKED_UP,
        TURN1,
        FORWARD_1,
        FORWARD_2,
        BT_FOLLOW,
        TURN2
    } robot_state = RobotState::INITIAL;

    int counter = 0; //counter

    //Forward movers
    float forw_1;
    float forw_2;

    // control algorithm in robot velocities
    Eigen::Vector2f robot_coord;
                    // proportional angle controller

    // map robot velocities to wheel velocities in rad/sec
    Eigen::Vector2f wheel_speed;

    //raw data tracker
    int lval;

    //Rotation Stuff
    const float angle_threshold = 0.01f;
    int turn_center = 0;

    // mechanical button
    DigitalIn mechanical_button(PC_5); // create DigitalIn object to evaluate mechanical button, you
                                    // need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);    // sets pullup between pin and 3.3 V, so that there
                                    // is a defined potential

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        lval = sensorBar.getRaw();
        printf("lval: %d \n", lval);

        // only update sensor bar angle if a led is triggered
        if (sensorBar.isAnyLedActive()) {
            angle = sensorBar.getAvgAngleRad();
        }

        if (do_execute_main_task) {
           
            enable_motors = 1; //Enable Motor

            // state machine
                switch (robot_state) {
                    case RobotState::INITIAL:
                        //Intial starting state
                        printf("INITIAL\n");

                        robot_state = RobotState::LINE_FOLLOW;

                        break;

                    case RobotState::LINE_FOLLOW:
                        //Main line following state, enter all other states from here
                        printf("lval: %d \n", lval);
                        printf("LINE_FOLLOW\n");
                        
                        // control algorithm in robot velocities
                        robot_coord = {0.5f * wheel_vel_max * r1_wheel, // half of the max. forward velocity
                                                    Kp * angle};                     // proportional angle controller

                        // map robot velocities to wheel velocities in rad/sec
                        wheel_speed = Cwheel2robot.inverse() * robot_coord;

                        // setpoints for the dc-motors in rps
                        motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PI)); // set a desired speed for speed controlled dc motors M1
                        motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PI)); // set a desired speed for speed controlled dc motors M2

                        if (mechanical_button.read()) {
                            robot_state = RobotState::BRICKED_UP;
                        } 
                        break;

                    case RobotState::BRICKED_UP:
                        printf("BRICKED_UP\n");

                        forw_1 = motor_M1.getRotation() - 0.5f;
                        forw_2 = motor_M2.getRotation() - 0.5f;

                        motor_M1.setRotation(forw_1);
                        motor_M2.setRotation(forw_2);

                        robot_state = RobotState::TURN1;

                        break;

                    case RobotState::TURN1:
                        printf("TURN1\n");

                        if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                            (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                            
                            forw_1 = motor_M1.getRotation() + 0.4f;
                            forw_2 = motor_M2.getRotation() - 0.4f;

                            motor_M1.setRotation(forw_1);
                            motor_M2.setRotation(forw_2);

                            robot_state = RobotState::FORWARD_1;
                        }

                        break;

                    case RobotState::FORWARD_1:
                        printf("FORWARD\n");
                        
                        if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                            (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                            
                            forw_1 = motor_M1.getRotation() + 2.5f;
                            forw_2 = motor_M2.getRotation() + 2.5f;

                            motor_M1.setRotation(forw_1);
                            motor_M2.setRotation(forw_2);

                            robot_state = RobotState::TURN2;
                        }

                        break;

                    case RobotState::TURN2:
                        printf("TURN2\n");

                        if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                            (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                            
                            forw_1 = motor_M1.getRotation() - 0.8f;
                            forw_2 = motor_M2.getRotation() + 0.8f;

                            motor_M1.setRotation(forw_1);
                            motor_M2.setRotation(forw_2);

                            robot_state = RobotState::FORWARD_2;
                        }
                        break;
                    
                    case RobotState::FORWARD_2:
 
                        if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                            (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {
                            
                            forw_1 = motor_M1.getRotation() + 2.0f;
                            forw_2 = motor_M2.getRotation() + 2.0f;

                            motor_M1.setRotation(forw_1);
                            motor_M2.setRotation(forw_2);

                            robot_state = RobotState::BT_FOLLOW;
                        }

                        break;

                    case RobotState::BT_FOLLOW:
                        //Intial starting state
                        printf("BT_FOLLOW\n");

                        if ((fabs(motor_M1.getRotationTarget() - motor_M1.getRotation()) < angle_threshold) &&
                        (fabs(motor_M2.getRotationTarget() - motor_M2.getRotation()) < angle_threshold)) {

                            robot_state = RobotState::LINE_FOLLOW;
                        }

                        break;

                    default:

                        break; // do nothing
                }
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