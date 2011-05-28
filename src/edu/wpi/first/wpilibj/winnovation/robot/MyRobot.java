/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.winnovation.robot;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import edu.wpi.first.wpilibj.winnovation.utils.FixedGyro;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class MyRobot extends IterativeRobot {

    // controls
    private Joystick leftJoystick;
    private Joystick rightJoystick;
    private RobotDrive robotDrive;

    // sensors
    private Encoder lEncoder;
    private Encoder rEncoder;
    private Gyro gyro;

    // speed controllers
    private SpeedController lDriveCim1;
    private SpeedController lDriveCim2;
    private SpeedController rDriveCim1;
    private SpeedController rDriveCim2;

    private Solenoid lobster;
    private Solenoid gearbox;

    private Compressor compressor;

    private Localizer localizer;

    private Motion[] autonDriveMotions;
    private int autonState;

    private boolean lobsterButtonReleased = true;
    private boolean gearButtonReleased = true;


    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {

        // init controls
        leftJoystick = new Joystick(Constants.LeftJoystickPort);
        rightJoystick = new Joystick(Constants.RightJoystickPort);

        // init sensors
        lEncoder = new Encoder(Constants.LeftDriveEncoderAlphaCh,
                Constants.LeftDriveEncoderBetaCh, true, Encoder.EncodingType.k1X);
        rEncoder = new Encoder(Constants.RightDriveEncoderAlphaCh,
                Constants.RightDriveEncoderBetaCh, true, Encoder.EncodingType.k1X);
        lEncoder.setDistancePerPulse(Constants.LeftDriveDistancePerPulse);
        rEncoder.setDistancePerPulse(Constants.RightDriveDistancePerPulse);
        lEncoder.start();
        rEncoder.start();

        gyro = new FixedGyro(Constants.GyroCh);


        // speed controllers
        if(Constants.IsPracticeBot) {
            lDriveCim1 = new Victor(Constants.LeftDriveCim1Ch);
            lDriveCim2 = new Victor(Constants.LeftDriveCim2Ch);
            rDriveCim1 = new Victor(Constants.RightDriveCim1Ch);
            rDriveCim2 = new Victor(Constants.RightDriveCim2Ch);
        } else {
            lDriveCim1 = new Jaguar(Constants.LeftDriveCim1Ch);
            lDriveCim2 = new Jaguar(Constants.LeftDriveCim2Ch);
            rDriveCim1 = new Jaguar(Constants.RightDriveCim1Ch);
            rDriveCim2 = new Jaguar(Constants.RightDriveCim2Ch);
        }

        // solenoids
        /*lobster = new Solenoid(Constants.LobsterSlot, Constants.LobsterCh);
        gearbox = new Solenoid(Constants.GearboxSlot, Constants.GearboxCh);

        compressor = new Compressor(Constants.PressureSwitchSlot, Constants.PressureSwitchCh,
                Constants.CompressorRelaySlot, Constants.CompressorRelayCh);
        */
        
        robotDrive = new PIDRobotDrive(lEncoder, rEncoder, lDriveCim1, lDriveCim2, rDriveCim1, rDriveCim2);
        getWatchdog().setEnabled(false);

    }


    public void autonomousInit() {
        super.autonomousInit();
        localizer = new Localizer(gyro, lEncoder, rEncoder);
        localizer.reset();
        /*compressor.start();

        autonDriveMotions = new Motion[0];
        autonState = 0;*/

    }

    public void autonomousContinuous() {
        super.autonomousContinuous();
        localizer.update();
        Timer.delay(Constants.ContinuousInterval);
    }


    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

        if(autonState < autonDriveMotions.length) {
            autonDriveMotions[autonState].doMotion();
            if(autonDriveMotions[autonState].isDone())
                autonState++;
        }
        Timer.delay(Constants.PeriodicInterval);
    }


    public void teleopInit() {
        super.teleopInit();
        if(localizer == null)
            localizer = new Localizer(gyro, lEncoder, rEncoder);
        localizer.reset();
        //compressor.start();
    }

    public void teleopContinuous() {
        super.teleopContinuous();
        localizer.update();
        Timer.delay(Constants.ContinuousInterval);
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        
       robotDrive.tankDrive(leftJoystick, rightJoystick);

        /*
        // toggle lobsters
        if(leftJoystick.getButton(Constants.LobsterButton) && lobsterButtonReleased) {
           lobsterButtonReleased = false;
           lobster.set(!lobster.get());
        } else if(!leftJoystick.getButton(Constants.LobsterButton)) {
            lobsterButtonReleased = true;
        }

        // switch gears
        if(rightJoystick.getButton(Constants.GearboxButton) && gearButtonReleased) {
           gearButtonReleased = false;
           gearbox.set(!gearbox.get());
        } else if(!rightJoystick.getButton(Constants.GearboxButton)) {
            gearButtonReleased = true;
        }*/
        Timer.delay(Constants.PeriodicInterval);

    }
    
}
