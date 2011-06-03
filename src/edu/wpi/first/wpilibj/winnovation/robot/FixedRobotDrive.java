
package edu.wpi.first.wpilibj.winnovation.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * This corrects the joysticks being reversed
 *
 * @author Chris
 */
public class FixedRobotDrive extends RobotDrive {

    FixedRobotDrive(SpeedController frontLeftMotor, SpeedController rearLeftMotor,
            SpeedController frontRightMotor, SpeedController rearRightMotor) {
        super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    }

    public void tankDrive(GenericHID leftStick, GenericHID rightStick) {
        super.tankDrive(-leftStick.getY(), -rightStick.getY());
    }

}
