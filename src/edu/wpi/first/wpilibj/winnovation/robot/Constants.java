
package edu.wpi.first.wpilibj.winnovation.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Global constants and parameters should be stored here
 *
 * @author Chris
 */
public class Constants {
    
    // misc
    public static final boolean IS_PRACTICE_BOT = true;
    public static final boolean USE_GYRO = true;
    public static final boolean LOGGING_ENABLED = true;

    // intervals code runs at
    public static final double CONTINUOUS_INTERVAL = 0.01; // seconds
    public static final double PERIODIC_INTERVAL = 0.01; // seconds

    // compressor
    public static final int PRESSURE_SWITCH_SLOT = 0;
    public static final int PRESSURE_SWITCH_CH = 0;
    public static final int COMPRESSOR_RELAY_SLOT = 0;
    public static final int COMPRESSOR_RELAY_CH = 0;

    // sensors
    public static final int LEFT_DRIVE_ENCODER_A_CH = 3;
    public static final int LEFT_DRIVE_ENCODER_B_CH = 4;
    public static final int RIGHT_DRIVE_ENCODER_A_CH = 1;
    public static final int RIGHT_DRIVE_ENCODER_B_CH = 2;
    public static final int GYRO_CH = 2;

    // switches

    // controls
    public static final int LEFT_JOYSTICK_PORT = 1;
    public static final int RIGHT_JOYSTICK_PORT = 2;

    // speed controllers

    // drive motors
    public static final int LEFT_DRIVE_CIM_1_CH = 4;
    //public static final int LeftDriveCim1Slot = 6;
    public static final int LEFT_DRIVE_CIM_2_CH = 5;
    //public static final int LeftDriveCim2Slot = 6;
    public static final int RIGHT_DRIVE_CIM_1_CH = 1;
    //public static final int RightDriveCim1Slot = 6;
    public static final int RIGHT_DRIVE_CIM_2_CH = 2;
    //public static final int RightDriveCim2Slot = 6;

    // solenoids
    public static final int LOBSTER_SLOT = 0;
    public static final int LOBSTER_CH = 0;
    public static final int GEARBOX_SLOT = 0;
    public static final int GEARBOX_CH = 0;


        // drive
    public static final double HIGH_GEAR_SPEED = 13.0; // ft/s
    public static final double LOW_GEAR_SPEED = 6.0; // ft/s // is this right?
    public static final double MIN_DRIVE_SPEED = 0.7; // ft/s
    public static final double LEFT_DRIVE_DIST_PER_PULSE = 0.0086528; // feet per encoder tick
    public static final double RIGHT_DRIVE_DIST_PER_PULSE = 0.0068988; // feet per encoder tick
    

    // dimensions
    public static double WHEEL_BASE_WIDTH = 28.0/12.0;//1.958;//23.50/12.0; // distance between the left and right wheels (ft)

    // Controls
    public static final Joystick.ButtonType LOBSTER_BUTTON = Joystick.ButtonType.kTrigger;
    public static final Joystick.ButtonType GEARBOX_BUTTON = Joystick.ButtonType.kTrigger;


}
