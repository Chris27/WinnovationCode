package edu.wpi.first.wpilibj.winnovation.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.winnovation.utils.LinearVictor;
import edu.wpi.first.wpilibj.winnovation.utils.PIDTunable;
import edu.wpi.first.wpilibj.winnovation.utils.ThreadlessPID;

/**
 * Extends RobotDrive to accept velocity as input instead of power
 *
 * @author Chris
 */
public class PIDRobotDrive extends RobotDrive implements PIDTunable {

    private static PIDRobotDrive instance = null;
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
    private double Kp = 0.070 * pMulti;
    private double Ki = 0.005 * iMulti;
    private double Kd = 0.016 * dMulti;
    private final double CAP = 5.0; // caps the amount of effect the pid can tweak the motor inputs
    private double tSens = 1.5; // for cheesydrive

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
        lController.setOutputRange(-CAP, CAP);
        rController.setOutputRange(-CAP, CAP);
        lController.setSetpoint(0);
        rController.setSetpoint(0);

    }

    public static synchronized PIDRobotDrive getInstance() {
        if (instance == null) {
            // speed controllers
            SpeedController ldc1;
            SpeedController ldc2;
            SpeedController rdc1;
            SpeedController rdc2;
            if (Constants.IS_PRACTICE_BOT) {
                ldc1 = new LinearVictor(Constants.LEFT_DRIVE_CIM_1_CH);
                ldc2 = new LinearVictor(Constants.LEFT_DRIVE_CIM_2_CH);
                rdc1 = new LinearVictor(Constants.RIGHT_DRIVE_CIM_1_CH);
                rdc2 = new LinearVictor(Constants.RIGHT_DRIVE_CIM_2_CH);
            } else {
                ldc1 = new Jaguar(Constants.LEFT_DRIVE_CIM_1_CH);
                ldc2 = new Jaguar(Constants.LEFT_DRIVE_CIM_2_CH);
                rdc1 = new Jaguar(Constants.RIGHT_DRIVE_CIM_1_CH);
                rdc2 = new Jaguar(Constants.RIGHT_DRIVE_CIM_2_CH);
            }
            Localizer l = Localizer.getInstance();
            instance = new PIDRobotDrive(l, ldc1, ldc2, rdc1, rdc2);
        }
        return instance;
    }

    private double cap(double val) {
        if (val > 1) {
            val = 1;
        } else if (val < -1) {
            val = -1;
        }
        return val;
    }

    public void tankDrive(GenericHID leftStick, GenericHID rightStick, boolean isHighGear) {

        // put a deadband in the joystick
        double leftOut = -leftStick.getY();
        leftOut = (Math.abs(leftOut) < Constants.DRIVE_JOYSTICK_THRESH) ? 0 : leftOut;
        double rightOut = -rightStick.getY();
        rightOut = (Math.abs(rightOut) < Constants.DRIVE_JOYSTICK_THRESH) ? 0 : rightOut;

        this.tankDrive(leftOut, rightOut, isHighGear);
    }

    public void tankDrive(GenericHID leftStick, GenericHID rightStick) {
        tankDrive(leftStick, rightStick, true);
    }

    public void tankDrive(double lVal, double rVal) {
        tankDrive(lVal, rVal, true);
    }

    public void tankDrive(double lVal, double rVal, boolean isHighGear) {
        if (isHighGear) {
            double leftVel = lVal * Constants.HIGH_GEAR_SPEED;
            double rightVel = rVal * Constants.HIGH_GEAR_SPEED;
            tankDriveAtVelocity(leftVel, rightVel);
        } else {
            double leftVel = lVal * Constants.HIGH_GEAR_SPEED;
            double rightVel = rVal * Constants.HIGH_GEAR_SPEED;
            tankDriveAtVelocity(leftVel, rightVel);
        }
    }

    public void cheesyDrive(double throttle, double wheel) {
        cheesyDrive(throttle, wheel, false);
    }

    public void cheesyDrive(double throttle, double wheel, boolean quickTurn) {
        double angular_power = 0.0;
        double overPower = 0.0;
        double sensitivity = tSens;
        double rPower = 0.0;
        double lPower = 0.0;

        if (quickTurn) {
            overPower = 1.0;
            sensitivity = 1.0;
            angular_power = wheel;
        } else {
            overPower = 0.0;
            angular_power = Math.abs(throttle) * wheel * sensitivity;
        }

        rPower = lPower = throttle;
        lPower += angular_power;
        rPower -= angular_power;

        if (lPower > 1.0) {
            rPower -= overPower * (lPower - 1.0);
            lPower = 1.0;
        } else if (rPower > 1.0) {
            lPower -= overPower * (rPower - 1.0);
            rPower = 1.0;
        } else if (lPower < -1.0) {
            rPower += overPower * (-1.0 - lPower);
            lPower = -1.0;
        } else if (rPower < -1.0) {
            lPower += overPower * (-1.0 - rPower);
            rPower = -1.0;
        }

        tankDrive(lPower, rPower);

    }

    /**
     *
     * @param leftVel linear velocity of left wheels
     * @param rightVel linear velocity of right wheels
     */
    public void tankDriveAtVelocity(double leftVel, double rightVel) {

        double leftOut = leftVel / Constants.HIGH_GEAR_SPEED;
        double rightOut = -rightVel / Constants.HIGH_GEAR_SPEED;

        double leftError = leftVel - localizer.getLVel();
        double rightError = rightVel - localizer.getRVel();
        double leftCorrection = lController.calculate(leftError) * Math.abs(leftOut);
        double rightCorrection = -rController.calculate(rightError) * Math.abs(rightOut);


        double lTweaked = cap(leftOut + leftCorrection);
        double rTweaked = cap(rightOut + rightCorrection);

        // to avoid "seizure mode" don't allow the correction to change the sign of the output
//        leftOut = sameSign(lTweaked, leftOut) ? lTweaked : 0;
//        rightOut = sameSign(rTweaked, rightOut) ? rTweaked : 0;


        lCim1.set(leftOut);
        lCim2.set(leftOut);
        rCim1.set(rightOut);
        rCim2.set(rightOut);

        if (Constants.LOGGING_ENABLED) {
            SmartDashboard.log(leftVel, "desired left ft/s");
            SmartDashboard.log(rightVel, "desired right ft/s");
            SmartDashboard.log(lCim1.get(), "left cim ouput");
            SmartDashboard.log(rCim1.get(), "right cim output");
            SmartDashboard.log((leftCorrection / Math.abs(leftOut) / CAP), "left correction");
            SmartDashboard.log((rightCorrection / Math.abs(rightOut) / CAP), "right correction");
        }
    }

    public void drive(double vel, double curve) {
        tankDrive(vel - curve * vel * Constants.WHEEL_BASE_WIDTH / 2.0, vel + curve * vel * Constants.WHEEL_BASE_WIDTH / 2.0);
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
