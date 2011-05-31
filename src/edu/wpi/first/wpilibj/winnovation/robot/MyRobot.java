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
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.winnovation.motions.DriveToMotion;
import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import edu.wpi.first.wpilibj.winnovation.motions.TurnToMotion;
import edu.wpi.first.wpilibj.winnovation.utils.FixedGyro;
import edu.wpi.first.wpilibj.winnovation.utils.LinearVictor;
import edu.wpi.first.wpilibj.winnovation.utils.PIDTunable;
import edu.wpi.first.wpilibj.winnovation.utils.PIDTuner;
import edu.wpi.first.wpilibj.winnovation.utils.PulseTriggerBoolean;

/**
 * The main robot routine
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
    private boolean lobsterButtonReleased = true;
    private boolean gearButtonReleased = true;
    private PIDTuner pidTuner;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {

        // init controls
        leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_PORT);
        rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_PORT);

        // init sensors
        lEncoder = new Encoder(Constants.LEFT_DRIVE_ENCODER_A_CH,
                Constants.LEFT_DRIVE_ENCODER_B_CH, true, Encoder.EncodingType.k1X);
        rEncoder = new Encoder(Constants.RIGHT_DRIVE_ENCODER_A_CH,
                Constants.RIGHT_DRIVE_ENCODER_B_CH, true, Encoder.EncodingType.k1X);
        lEncoder.setDistancePerPulse(Constants.LEFT_DRIVE_DIST_PER_PULSE);
        rEncoder.setDistancePerPulse(Constants.RIGHT_DRIVE_DIST_PER_PULSE);
        lEncoder.start();
        rEncoder.start();

        gyro = new FixedGyro(Constants.GYRO_CH);


        // speed controllers
        if (Constants.IS_PRACTICE_BOT) {
            lDriveCim1 = new LinearVictor(Constants.LEFT_DRIVE_CIM_1_CH);
            lDriveCim2 = new LinearVictor(Constants.LEFT_DRIVE_CIM_2_CH);
            rDriveCim1 = new LinearVictor(Constants.RIGHT_DRIVE_CIM_1_CH);
            rDriveCim2 = new LinearVictor(Constants.RIGHT_DRIVE_CIM_2_CH);
        } else {
            lDriveCim1 = new Jaguar(Constants.LEFT_DRIVE_CIM_1_CH);
            lDriveCim2 = new Jaguar(Constants.LEFT_DRIVE_CIM_2_CH);
            rDriveCim1 = new Jaguar(Constants.RIGHT_DRIVE_CIM_1_CH);
            rDriveCim2 = new Jaguar(Constants.RIGHT_DRIVE_CIM_2_CH);
        }

        // solenoids
        /*lobster = new Solenoid(Constants.LobsterSlot, Constants.LobsterCh);
        gearbox = new Solenoid(Constants.GearboxSlot, Constants.GearboxCh);

        compressor = new Compressor(Constants.PressureSwitchSlot, Constants.PressureSwitchCh,
        Constants.CompressorRelaySlot, Constants.CompressorRelayCh);
         */

        getWatchdog().setEnabled(false);

    }
    private Motion testMotion;
    private Joystick gp;
    private PulseTriggerBoolean incTog;
    private PulseTriggerBoolean decTog;

    public void /*autonomousInit*/ teleopInit() {
        super.autonomousInit();
        localizer = new Localizer(gyro, lEncoder, rEncoder);
        localizer.reset();

        robotDrive = new PIDRobotDrive(localizer, lDriveCim1, lDriveCim2, rDriveCim1, rDriveCim2);
        testMotion = new DriveToMotion(robotDrive, 0.50, localizer, 12.0, 0.0);
        pidTuner = new PIDTuner(3, (PIDTunable) testMotion, 0.001, 0.001, 0.001, 27, 1, 10);

        gp = new Joystick(3);
        incTog = new PulseTriggerBoolean();
        decTog = new PulseTriggerBoolean();

        /*compressor.start();

        autonDriveMotions = new Motion[0];
        autonState = 0;*/

    }

    public void /*autonomousContinuous*/ teleopContinuous() {
        super.autonomousContinuous();
        localizer.update();
        Timer.delay(Constants.CONTINUOUS_INTERVAL);

    }

    /**
     * This function is called periodically during autonomous
     */
    public void /*autonomousPeriodic*/ teleopPeriodic() {

        incTog.set(gp.getRawButton(2));
        decTog.set(gp.getRawButton(4));

        if (incTog.get()) {
            Constants.WHEEL_BASE_WIDTH += 0.01;
            SmartDashboard.log(Constants.WHEEL_BASE_WIDTH, "wheel base width");
        }
        if (decTog.get()) {
            Constants.WHEEL_BASE_WIDTH -= 0.01;
            SmartDashboard.log(Constants.WHEEL_BASE_WIDTH, "wheel base width");
        }


        pidTuner.handle();
        if (pidTuner.reset()) {
            localizer.reset();
            testMotion = new DriveToMotion(robotDrive, 0.50, localizer, 12.0, 0.0);
            pidTuner = new PIDTuner(3, (PIDTunable) testMotion, 0.001, 0.001, 0.001, pidTuner.cp, pidTuner.ci, pidTuner.cd);
        }

        if (testMotion != null) {
            testMotion.doMotion();
            SmartDashboard.log(testMotion.isDone(), "test motion");
        }
        Timer.delay(Constants.PERIODIC_INTERVAL);

    }
//    public void teleopInit() {
//        super.teleopInit();
//        if(localizer == null) {
//            localizer = new Localizer(gyro, lEncoder, rEncoder);
//            localizer.reset();
//            robotDrive = new PIDRobotDrive(localizer, lDriveCim1, lDriveCim2, rDriveCim1, rDriveCim2);
//        }
//
//        // make sure auton test is dead
//        if(testMotion != null)
//            testMotion.abort();
//
//        //pidTuner = new PIDTuner(3, (PIDRobotDrive) robotDrive, 20, 14, 15);
//        //compressor.start();
//    }
//
//    public void teleopContinuous() {
//        super.teleopContinuous();
//        localizer.update();
//        Timer.delay(Constants.CONTINUOUS_INTERVAL);
//    }
//
//    /**
//     * This function is called periodically during operator control
//     */
//    public void teleopPeriodic() {
//
//       robotDrive.tankDrive(leftJoystick, rightJoystick);
//       //pidTuner.handle();
//        /*
//        // toggle lobsters
//        if(leftJoystick.getButton(Constants.LobsterButton) && lobsterButtonReleased) {
//           lobsterButtonReleased = false;
//           lobster.set(!lobster.get());
//        } else if(!leftJoystick.getButton(Constants.LobsterButton)) {
//            lobsterButtonReleased = true;
//        }
//
//        // switch gears
//        if(rightJoystick.getButton(Constants.GearboxButton) && gearButtonReleased) {
//           gearButtonReleased = false;
//           gearbox.set(!gearbox.get());
//        } else if(!rightJoystick.getButton(Constants.GearboxButton)) {
//            gearButtonReleased = true;
//        }*/
//        Timer.delay(Constants.PERIODIC_INTERVAL);
//
//    }
}
