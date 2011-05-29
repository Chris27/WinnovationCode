package edu.wpi.first.wpilibj.winnovation.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.winnovation.utils.ThreadlessPID;
/**
 * Extends RobotDrive to accept velocity as input instead of power
 *
 * @author Chris
 */
public class PIDRobotDrive extends RobotDrive {

    private ThreadlessPID lController;
    private ThreadlessPID rController;
    private SpeedController lCim1;
    private SpeedController lCim2;
    private SpeedController rCim1;
    private SpeedController rCim2;
    private Localizer localizer;
    

    // PID constants
    private double Kp = 0.070*18; // constants on right multipliers account for
    private double Ki = 0.005*9; // the robot being on carpet
    private double Kd = 0.016*15;



    public PIDRobotDrive(Localizer localizer,
            SpeedController lCim1, SpeedController lCim2, SpeedController rCim1,
            SpeedController rCim2) {

        super(lCim1, lCim2, rCim1, rCim2);
        this.lCim1 = lCim1;
        this.lCim2 = lCim2;
        this.rCim1 = rCim1;
        this.rCim2 = rCim2;
        this.localizer = localizer;

        lController = new ThreadlessPID(Kp, Ki, Kd);
        rController = new ThreadlessPID(Kp, Ki, Kd);

        lController.setInputRange(-0.25*Constants.MAX_DRIVE_SPEED, 0.25*Constants.MAX_DRIVE_SPEED);
        lController.setOutputRange(-0.75, 0.75);
        rController.setInputRange(-0.25*Constants.MAX_DRIVE_SPEED, 0.25*Constants.MAX_DRIVE_SPEED);
        rController.setOutputRange(-0.75, 0.75);
        lController.setSetpoint(0);
        rController.setSetpoint(0);
        
    }
    
    private double cap(double val) {
        if(val > 1)
            val = 1;
        else if(val < -1)
            val = -1;
        return val;
    }


    public void tankDrive(GenericHID leftStick, GenericHID rightStick) {

        this.tankDrive(-leftStick.getY(), -rightStick.getY());
    }

    public void tankDrive(double lVal, double rVal) {
        double leftVel = -lVal*Constants.MAX_DRIVE_SPEED;
        double rightVel = -rVal*Constants.MAX_DRIVE_SPEED;
        tankDriveAtVelocity(leftVel, rightVel);
    }

    /**
     *
     * @param leftVel linear velocity of left wheels
     * @param rightVel linear velocity of right wheels
     */
    public void tankDriveAtVelocity(double leftVel, double rightVel) {

        double leftOut = leftVel/Constants.MAX_DRIVE_SPEED;
        double rightOut = -rightVel/Constants.MAX_DRIVE_SPEED;

        double leftError = leftVel - localizer.getLVel();
        double rightError = rightVel - localizer.getRVel();
        double leftCorrection = lController.calculate(leftError);
        double rightCorrection = -rController.calculate(rightError);

        leftOut = cap(leftOut + leftCorrection);
        rightOut = cap(rightOut + rightCorrection);

         // deadzone
        if(Math.abs(leftVel) < Constants.MIN_DRIVE_SPEED)
            leftOut = 0;
        if(Math.abs(rightVel) < Constants.MIN_DRIVE_SPEED)
            rightOut = 0;
  
        lCim1.set(leftOut);
        lCim2.set(leftOut);
        rCim1.set(rightOut);
        rCim2.set(rightOut);

        SmartDashboard.log(leftVel, "desired left ft/s");
        SmartDashboard.log(rightVel, "desired right ft/s");
        SmartDashboard.log(lCim1.get(), "left cim ouput");
        SmartDashboard.log(rCim1.get(), "right cim output");
        SmartDashboard.log(leftCorrection, "left correction");
        SmartDashboard.log(rightCorrection, "right correction");
    }

    public void drive(double vel, double curve) {
        tankDrive(vel - curve*vel*Constants.WHEEL_BASE_WIDTH/2.0, vel + curve*vel*Constants.WHEEL_BASE_WIDTH/2.0);
    }


}
