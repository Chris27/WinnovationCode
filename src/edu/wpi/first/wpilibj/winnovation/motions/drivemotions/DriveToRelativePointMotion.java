package edu.wpi.first.wpilibj.winnovation.motions.drivemotions;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SmartDashboard;
import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import edu.wpi.first.wpilibj.winnovation.config.Constants;
import edu.wpi.first.wpilibj.winnovation.config.PIDValues;
import edu.wpi.first.wpilibj.winnovation.robot.Localizer;
import edu.wpi.first.wpilibj.winnovation.robot.PIDRobotDrive;
import edu.wpi.first.wpilibj.winnovation.utils.Angle;
import edu.wpi.first.wpilibj.winnovation.utils.PIDTunable;
import edu.wpi.first.wpilibj.winnovation.utils.ThreadlessPID;
import edu.wpi.first.wpilibj.winnovation.utils.Trig;

/**
 * This motion will drive the robot to a point (x,y) relative to the current position
 * of the robot.  The x-axis is forward and the y-axis is too the left.  PID control
 * monitors the trajectory of the robot and tries to stop the robot after the
 * correct distance.
 * 
 * @author Chris
 */
public class DriveToRelativePointMotion implements Motion, PIDTunable {

    public static final double DEFAULT_ALLOWABLE_ERROR = 0.3; // ft of allowable error
    public static final double EXIT_SPEED = 0.7; // ft/s robot is allowed to be done if overshoot==false
    // PID values that control distance monitoring
    private double distP = PIDValues.DIST_CORRECT_P; //0.005 * 130;
    private double distI = PIDValues.DIST_CORRECT_I; //0.005 * 3;
    private double distD = PIDValues.DIST_CORRECT_D; //0.005 * 35;
    // PID values that control trajectory monitoring
    private double rotP = PIDValues.HEADING_CORRECT_P; //0.05 * 185;
    private double rotI = PIDValues.HEADING_CORRECT_I; //0.005 * 10;
    private double rotD = PIDValues.HEADING_CORRECT_D; //0.05 * 15;
    private double rotCap = PIDValues.HEADING_CORRECT_CAP; //0.3;
    private ThreadlessPID distPID;
    private ThreadlessPID rotPID;
    private RobotDrive robotDrive;
    private Localizer localizer;
    private double initLDist;
    private double initRDist;
    private double initHeading;
    private double arcLength;
    private double arcRadians;
    private double curv;
    private boolean overshoot;
    private double allowedError;
    private boolean hasInitialized = false;
    private boolean isDone;

    /**
     *
     * @param x desired x position (relative)
     * @param y desired y position (relative)
     * @param speed [0,1] max speed
     * @param error allowable error (feet)
     * @param allowOvershoot set true if you don't want the robot to oscillate around the targe
     */
    public DriveToRelativePointMotion(double x, double y, double speed, double error, boolean allowOvershoot) {

        this.robotDrive = PIDRobotDrive.getInstance();
        this.localizer = Localizer.getInstance();
        this.allowedError = error;
        this.overshoot = allowOvershoot;

        distPID = new ThreadlessPID(distP, distI, distD);
        distPID.setOutputRange(-Math.abs(speed), Math.abs(speed));
        rotPID = new ThreadlessPID(rotP, rotI, rotD);
        rotPID.setOutputRange(-rotCap, rotCap);

        if (x == 0 && y == 0) {
            isDone = true;
        } else {
            isDone = false;
            curv = 2.0 * y / (x * x + y * y);

            // straight case
            if (curv == 0) {
                arcRadians = 0;
                arcLength = x;
            } else {
                double r = 1.0 / curv;
                arcRadians = sign(y) * Trig.atan2(x, r - y);
                arcLength = arcRadians * r;
                SmartDashboard.log(MathUtils.atan2(x, r - y), "atan2");
                SmartDashboard.log(arcRadians, "arcRadians");
                SmartDashboard.log(arcLength, "arcLength");
            }

        }


    }


    public DriveToRelativePointMotion(double x, double y, double speed, double error) {
        this(x, y, speed, error, false);
    }

    public DriveToRelativePointMotion(double x, double y, double speed, boolean allowOvershoot) {
        this(x, y, speed, DEFAULT_ALLOWABLE_ERROR, allowOvershoot);
    }

    public DriveToRelativePointMotion(double x, double y, double speed) {
        this(x, y, speed, DEFAULT_ALLOWABLE_ERROR, false);
    }

    private double getDesiredHeading(double dist) {
        return (dist / arcLength) * arcRadians + initHeading;
    }

        // don't want to grab the current position until it is time for this motion to start
    private void init() {
        initLDist = localizer.getLDist();
        initRDist = localizer.getRDist();
        initHeading = localizer.getTh();
        hasInitialized = true;
    }

    private int sign(double x) {
        return x >= 0 ? 1 : -1;
    }

    public boolean isDone() {
        return isDone;
    }

    public void doMotion() {

        if (!hasInitialized) {
            init();
        }

        if (isDone) {
            abort();
            return;
        }

        // get distance along arc
        double dist = ((localizer.getLDist() - initLDist) + (localizer.getRDist() - initRDist)) / 2.0;
        double error = arcLength - dist;

        // if error is small and robot is coming to a stop then we are done
        if (Math.abs(error) < allowedError && (overshoot || Math.abs(localizer.getVel()) < EXIT_SPEED)) {
            abort();
            return;
        }

        // get base speeds
        double v = distPID.calculate(error);
        double leftSpeed = v - v * curv * Constants.WHEEL_BASE_WIDTH / 2.0;
        double rightSpeed = v + v * curv * Constants.WHEEL_BASE_WIDTH / 2.0;

        // correct heading
        error = Angle.normalize(getDesiredHeading(dist) - localizer.getTh());
        double correction = rotPID.calculate(error);
        if (localizer.getVel() >= 0) {
            leftSpeed -= correction;
        } else {
            rightSpeed += correction;
        }

        // set output speeds
        robotDrive.tankDrive(leftSpeed, rightSpeed);

    }

    public void abort() {
        isDone = true;
        robotDrive.tankDrive(0, 0);
    }

    public double getKp() {
        return rotP;
    }

    public double getKi() {
        return rotI;
    }

    public double getKd() {
        return rotD;
    }

    public void setKp(double Kp) {
        this.rotP = Kp;
        rotPID.setP(Kp);
    }

    public void setKi(double Ki) {
        this.rotI = Ki;
        rotPID.setI(Ki);
    }

    public void setKd(double Kd) {
        this.rotD = Kd;
        rotPID.setD(Kd);
    }

    public void setPID(double Kp, double Ki, double Kd) {
        setKp(Kp);
        setKi(Ki);
        setKd(Kd);
    }
}
