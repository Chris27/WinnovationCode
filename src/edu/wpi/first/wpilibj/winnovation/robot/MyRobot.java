package edu.wpi.first.wpilibj.winnovation.robot;

import edu.wpi.first.wpilibj.winnovation.config.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.winnovation.autonomous.AutonomousMode;
import edu.wpi.first.wpilibj.winnovation.autonomous.AutonomousSelector;
import edu.wpi.first.wpilibj.winnovation.controls.AuxiliaryControl;
import edu.wpi.first.wpilibj.winnovation.controls.Control;
import edu.wpi.first.wpilibj.winnovation.controls.DriverControl;

/**
 * The main robot routine.  Everything starts here.  You shouldn't need to modify
 * this file much as to keep the code organized, you should implement stuff elsewhere
 */
public class MyRobot extends IterativeRobot {

    private Localizer localizer;
    private Control driverControl;
    private Control auxiliaryControl;
    private Compressor compressor;
    private AutonomousSelector autoSelect;
    private AutonomousMode autoMode;

    /**
     * This method is called once when the robot is first started up
     */
    public void robotInit() {
//        compressor = new Compressor(Constants.PRESSURE_SWITCH_SLOT, Constants.PRESSURE_SWITCH_CH,
//        Constants.COMPRESSOR_RELAY_SLOT, Constants.COMPRESSOR_RELAY_CH);
        getWatchdog().setEnabled(false);
    }

    /**
     * This method is called once before autonomous begins
     */
    public void autonomousInit() {
        super.autonomousInit();
        localizer = Localizer.getInstance();
        localizer.reset();
        //compressor.start();
        autoSelect = AutonomousSelector.getInstance();
        autoMode = autoSelect.getAutonomousMode();
    }

    /**
     * This method is called periodically (very quick interval) during autonomous
     */
    public void autonomousContinuous() {
        super.autonomousContinuous();
        // update the robot position
        localizer.update();
        Timer.delay(Constants.CONTINUOUS_INTERVAL);
    }

    /**
     * This method is called periodically (slower interval) during autonomous
     */
    public void autonomousPeriodic() {
        if (autoMode != null) {
            autoMode.run();
        }
        Timer.delay(Constants.PERIODIC_INTERVAL);
    }

    /**
     * This method is called once before teleop begins
     */
    public void teleopInit() {
        super.teleopInit();
        localizer = Localizer.getInstance();
        driverControl = DriverControl.getInstance();
        auxiliaryControl = AuxiliaryControl.getInstance();
        //compressor.start();
    }

    /**
     * This method is called periodically (very quick interval) during teleop
     */
    public void teleopContinuous() {
        super.teleopContinuous();
        localizer.update();
        Timer.delay(Constants.CONTINUOUS_INTERVAL);
    }

    /**
     * This method is called periodically (very quick interval) during teleop
     */
    public void teleopPeriodic() {
        // get control inputs and handle appropriately
        driverControl.handle();
        auxiliaryControl.handle();
        Timer.delay(Constants.PERIODIC_INTERVAL);
    }
}
