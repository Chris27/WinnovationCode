
package edu.wpi.first.wpilibj.winnovation.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.winnovation.robot.AuxiliaryDrive;
import edu.wpi.first.wpilibj.winnovation.config.Constants;
import edu.wpi.first.wpilibj.winnovation.robot.PIDRobotDrive;
import edu.wpi.first.wpilibj.winnovation.utils.PulseTriggerBoolean;

/**
 * WORK IN PROGRESS
 *
 * Reads in the driver controls and outputs the appropriate commands to the drive
 * train.
 *
 * todo: gear shifting, lobstering, minibot deploy
 *
 * @author Chris
 */
public class DriverControl implements Control {

    private static DriverControl instance = null;

    private Joystick leftJoystick;
    private Joystick rightJoystick;
    private PIDRobotDrive robotDrive;
    private AuxiliaryDrive auxiliaryDrive;

    private PulseTriggerBoolean shiftGears;
    private PulseTriggerBoolean reverseDrive;
    private PulseTriggerBoolean lockStraight;
    private PulseTriggerBoolean lobster;

    private boolean isHighGear = true;
    private boolean isReverseDrive = false;

    public DriverControl() {
        leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_PORT);
        rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_PORT);
        robotDrive = PIDRobotDrive.getInstance();
        auxiliaryDrive = AuxiliaryDrive.getInstance();
        shiftGears = new PulseTriggerBoolean();
        reverseDrive = new PulseTriggerBoolean();
        lockStraight = new PulseTriggerBoolean();
    }

    public static DriverControl getInstance() {
        if(instance == null) {
            instance = new DriverControl();
        }

        return instance;
    }

    public void handle() {
        
        // set the pulse triggers by reading the respective buttons


        robotDrive.tankDrive(leftJoystick, rightJoystick, isHighGear, isReverseDrive);
    }

}
