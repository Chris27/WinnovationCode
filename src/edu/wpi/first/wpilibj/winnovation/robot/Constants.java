/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 *
 * @author Chris
 */
public class Constants {
    
    // misc
    public static final boolean IsPracticeBot = true;
    public static final boolean UseGyro = false;

    // intervals code runs at
    public static final double ContinuousInterval = 0.01; // seconds
    public static final double PeriodicInterval = 0.01; // seconds

    // compressor
    public static final int PressureSwitchSlot = 0;
    public static final int PressureSwitchCh = 0;
    public static final int CompressorRelaySlot = 0;
    public static final int CompressorRelayCh = 0;


    // sensors
    public static final int LeftDriveEncoderAlphaCh = 3;
    public static final int LeftDriveEncoderBetaCh = 4;
    public static final int RightDriveEncoderAlphaCh = 1;
    public static final int RightDriveEncoderBetaCh = 2;
    public static final int GyroCh = 2;

    // switches

    // controls
    public static final int LeftJoystickPort = 1;
    public static final int RightJoystickPort = 2;

    // speed controllers

    // drive motors
    public static final int LeftDriveCim1Ch = 4;
    //public static final int LeftDriveCim1Slot = 6;
    public static final int LeftDriveCim2Ch = 5;
    //public static final int LeftDriveCim2Slot = 6;
    public static final int RightDriveCim1Ch = 1;
    //public static final int RightDriveCim1Slot = 6;
    public static final int RightDriveCim2Ch = 2;
    //public static final int RightDriveCim2Slot = 6;

    // solenoids
    public static final int LobsterSlot = 0;
    public static final int LobsterCh = 0;
    public static final int GearboxSlot = 0;
    public static final int GearboxCh = 0;


        // drive
    public static final double MaxDriveSpeed = 13.0; // ft/s
    public static final double MinDriveSpeed = 0.7; // ft/s
    public static final double LeftDriveDistancePerPulse = 0.0086528; // feet per encoder tick
    public static final double RightDriveDistancePerPulse = 0.0068988; // feet per encoder tick
    

    // dimensions
    public static final double WheelBaseWidth = 24.75/12.0; // distance between the left and right wheels (ft)
    //public static double WheelRadius = 1.75/12.0;
    //public static double LobsterWheelRadius = 1.75/12.0;



    // encoders
    //public static double EncoderTicksPerRev = 360;


    // Controls
    public static final Joystick.ButtonType LobsterButton = Joystick.ButtonType.kTrigger;
    public static final Joystick.ButtonType GearboxButton = Joystick.ButtonType.kTrigger;


}
