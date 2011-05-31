package edu.wpi.first.wpilibj.winnovation.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.winnovation.utils.PIDTunable;
import edu.wpi.first.wpilibj.winnovation.utils.ThreadlessPID;
/**
 * Extends RobotDrive to accept velocity as input instead of power
 *
 * @author Chris
 */
public class PIDRobotDrive extends RobotDrive implements PIDTunable{

    private ThreadlessPID lController;
    private ThreadlessPID rController;
    private SpeedController lCim1;
    private SpeedController lCim2;
    private SpeedController rCim1;
    private SpeedController rCim2;
    private Localizer localizer;
    

    // PID constants
    // these account for the wheels bearing a load
    private double pMulti = 25;
    private double iMulti = 29;//18;
    private double dMulti = 20;
    // pid values
    private double Kp = 0.070*pMulti;
    private double Ki = 0.005*iMulti;
    private double Kd = 0.016*dMulti;

    private final double CAP = 5.0; // caps the amount of effect the pid can tweak the motor inputs


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
        lController.setOutputRange(-CAP, CAP);
        rController.setInputRange(-0.25*Constants.MAX_DRIVE_SPEED, 0.25*Constants.MAX_DRIVE_SPEED);
        rController.setOutputRange(-CAP, CAP);
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

    private boolean sameSign(double a, double b) {
        return ((a >= 0 && b >= 0) || (a <= 0 && b <= 0));
    }


    public void tankDrive(GenericHID leftStick, GenericHID rightStick) {

        this.tankDrive(-leftStick.getY(), -rightStick.getY());
    }

    public void tankDrive(double lVal, double rVal) {
        double leftVel = lVal*Constants.MAX_DRIVE_SPEED;
        double rightVel = rVal*Constants.MAX_DRIVE_SPEED;
        tankDriveAtVelocity(leftVel, rightVel);
    }

    public void cheesyDrive(double power, double curv) {
        cheesyDrive(power, curv, false);
    }

    public void cheesyDrive(double power, double curv, boolean quickTurn) {
        curv = cap(curv);
        power = cap(power);

        if(quickTurn) {

        } else {
            double left = curv;
            double right = 1 - curv;


            // todo
        }

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
        double leftCorrection = lController.calculate(leftError)*Math.abs(leftOut);
        double rightCorrection = -rController.calculate(rightError)*Math.abs(rightOut);

        // to avoid "seizure mode" don't allow the correction to change the sign of the output
        double lTweaked = cap(leftOut + leftCorrection);
        double rTweaked = cap(rightOut + rightCorrection);
        leftOut = sameSign(lTweaked, leftOut) ? lTweaked : 0;
        rightOut = sameSign(rTweaked, rightOut) ? rTweaked : 0;

         // deadzone
        if(Math.abs(leftVel) < Constants.MIN_DRIVE_SPEED)
            leftOut = 0;
        if(Math.abs(rightVel) < Constants.MIN_DRIVE_SPEED)
            rightOut = 0;

        lCim1.set(leftOut);
        lCim2.set(leftOut);
        rCim1.set(rightOut);
        rCim2.set(rightOut);

        if(Constants.LOGGING_ENABLED) {
            SmartDashboard.log(leftVel, "desired left ft/s");
            SmartDashboard.log(rightVel, "desired right ft/s");
            SmartDashboard.log(lCim1.get(), "left cim ouput");
            SmartDashboard.log(rCim1.get(), "right cim output");
            SmartDashboard.log((leftCorrection/Math.abs(leftOut)/CAP), "left correction");
            SmartDashboard.log((rightCorrection/Math.abs(rightOut)/CAP), "right correction");
        }
    }

    public void drive(double vel, double curve) {
        tankDrive(vel - curve*vel*Constants.WHEEL_BASE_WIDTH/2.0, vel + curve*vel*Constants.WHEEL_BASE_WIDTH/2.0);
    }

    public double getKp() {
        return Kp;
    }

    public double getKi() {
        return Ki;
    }

    public double getKd() {
        return Kd;
    }

    public void setKp(double Kp) {
        this.Kp = Kp;
        lController.setP(Kp);
        rController.setP(Kp);

    }

    public void setKi(double Ki) {
        this.Ki = Ki;
        lController.setI(Ki);
        rController.setI(Ki);
    }

    public void setKd(double Kd) {
        this.Kd = Kd;
        lController.setD(Kd);
        rController.setD(Kd);
    }

    public void setPID(double Kp, double Ki, double Kd) {
        setKp(Kp);
        setKi(Ki);
        setKd(Kd);
    }


}
