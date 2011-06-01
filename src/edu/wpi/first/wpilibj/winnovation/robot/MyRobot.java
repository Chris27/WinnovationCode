/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.winnovation.robot;

import edu.wpi.first.wpilibj.winnovation.config.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.winnovation.motions.drivemotions.DriveToPointRelativeMotion;
import edu.wpi.first.wpilibj.winnovation.motions.Motion;
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
    private PIDRobotDrive robotDrive;

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
        localizer = Localizer.getInstance();
        localizer.reset();
        robotDrive = PIDRobotDrive.getInstance();

        testMotion = new DriveToPointRelativeMotion(7.0, -3.0, 0.80);
        pidTuner = new PIDTuner(3, (PIDTunable) testMotion, 0.05, 0.005, 0.05, 40, 10, 0);

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
            Constants.WHEEL_BASE_WIDTH += 0.05;
            SmartDashboard.log(Constants.WHEEL_BASE_WIDTH, "wheel base width");
        }
        if (decTog.get()) {
            Constants.WHEEL_BASE_WIDTH -= 0.05;
            SmartDashboard.log(Constants.WHEEL_BASE_WIDTH, "wheel base width");
        }


        pidTuner.handle();
        if (pidTuner.reset()) {
            localizer.reset();
            testMotion = new DriveToPointRelativeMotion(7.0, -3.0, 0.80);
            pidTuner = new PIDTuner(3, (PIDTunable) testMotion, 0.05, 0.005, 0.05, pidTuner.cp, pidTuner.ci, pidTuner.cd);
        }

        if (testMotion != null) {
            testMotion.doMotion();
            SmartDashboard.log(testMotion.isDone(), "test motion");
        }
        Timer.delay(Constants.PERIODIC_INTERVAL);

    }
//    public void teleopInit() {
//        super.teleopInit();
//        localizer = Localizer.getInstance();
//        robotDrive = PIDRobotDrive.getInstance();
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
