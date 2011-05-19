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
    public static final boolean isPracticeBot = true;

    // compressor
    public static final int PressureSwitchSlot = 0;
    public static final int PressureSwitchCh = 0;
    public static final int CompressorRelaySlot = 0;
    public static final int CompressorRelayCh = 0;


    // sensors
    public static final int LeftDriveEncoderAlphaCh = 0;
    public static final int LeftDriveEncoderBetaCh = 0;
    public static final int RightDriveEncoderAlphaCh = 0;
    public static final int RightDriveEncoderBetaCh = 0;
    public static final int GyroSlot = 0;
    public static final int GyroCh = 0;

    // switches

    // controls
    public static final int LeftJoystickPort = 0;
    public static final int RightJoystickPort = 0;

    // speed controllers

    // drive motors
    public static final int LeftDriveCim1Ch = 0;
    public static final int LeftDriveCim1Slot = 0;
    public static final int LeftDriveCim2Ch = 0;
    public static final int LeftDriveCim2Slot = 0;
    public static final int RightDriveCim1Ch = 0;
    public static final int RightDriveCim1Slot = 0;
    public static final int RightDriveCim2Ch = 0;
    public static final int RightDriveCim2Slot = 0;

    // solenoids
    public static final int LobsterSlot = 0;
    public static final int LobsterCh = 0;
    public static final int GearboxSlot = 0;
    public static final int GearboxCh = 0;


    // drive
    public static double MaxDriveSpeed = 12.0;

    // dimensions
    public static double WheelBaseRadius = 26.0/12.0;
    public static double WheelRadius = 4.0;

    // encoders
    public static double EncoderTicksPerRev = 360;


    // Controls
    public static Joystick.ButtonType LobsterButton = Joystick.ButtonType.kTrigger;
    public static Joystick.ButtonType GearboxButton = Joystick.ButtonType.kTrigger;


}
