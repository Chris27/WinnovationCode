
package edu.wpi.first.wpilibj.winnovation.motions.drivemotions;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.winnovation.config.PIDValues;
import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import edu.wpi.first.wpilibj.winnovation.robot.Localizer;
import edu.wpi.first.wpilibj.winnovation.robot.PIDRobotDrive;
import edu.wpi.first.wpilibj.winnovation.utils.Angle;
import edu.wpi.first.wpilibj.winnovation.utils.PIDTunable;
import edu.wpi.first.wpilibj.winnovation.utils.ThreadlessPID;

/**
 * This motion turns the robot in place to the desired (absolute) heading
 *
 * @author Chris
 */
public class TurnAbsoluteMotion implements Motion, PIDTunable {

    private boolean done;
    private ThreadlessPID pid;
    private RobotDrive robotDrive;
    private Localizer localizer;
    private double target; // in radians


    public static final double DEFAULT_ERROR = 2.0*Math.PI/180.0;
    public static final double EXIT_SPEED = 1.3; // ft/s
    public static final int MAX_OSCILLATIONS = 3; // limits the number of times robot is allowed to oscillate

    // PID that monitors the orientation of the robot
    private double Kp = PIDValues.SPIN_CORRECT_P; //0.005*55;
    private double Ki = PIDValues.SPIN_CORRECT_I; //0.005*2;
    private double Kd = PIDValues.SPIN_CORRECT_D; //0.005*7;


    private boolean overshoot;
    private int oscillations = 0;
    private int prevSign = 0;


    /**
     *
     * @param degrees [-180,180) degrees
     * @param speed speed of rotation [0,1]
     * @param error allowable error (feet)
     * @param allowOvershoot set true if you don't want the robot to oscillate
     */
    public TurnAbsoluteMotion(double degrees, double speed, double error, boolean allowOvershoot) {
        overshoot = allowOvershoot;
        done = false;
        pid = new ThreadlessPID(Kp, Ki, Kd);
        pid.setOutputRange(-speed, speed);
        pid.setSetpoint(0);
        pid.setTolerance(error);
        robotDrive = PIDRobotDrive.getInstance();
        localizer = Localizer.getInstance();
        target = Angle.normalize(degrees*Math.PI/180.0);
    }

    public TurnAbsoluteMotion(double degrees, double speed, boolean allowOvershoot) {
        this(degrees, speed, DEFAULT_ERROR, allowOvershoot);
    }

    public TurnAbsoluteMotion(double degrees, double speed, double error) {
        this(degrees, speed, error, false);
    }

    public TurnAbsoluteMotion(double degrees, double speed) {
        this(degrees, speed, DEFAULT_ERROR, false);
    }

    public boolean isDone() {
        return done;
    }
    
    private int sign(double x) {
        if(x >= 0)
            return 1;
        else
            return -1;
    }



    public void doMotion() {

        if(done) {
            abort();
            return;
        }

        double curHeading = Angle.normalize(localizer.getTh());
        double dif = target - curHeading;
        double error;

        // choose the shortest direction to turn
        if (dif > 0) {
            if (dif < Math.PI) {
                error = dif;
            } else {
                error = -(2*Math.PI - dif);
            }
        } else {
            if (Math.abs(dif) > Math.PI) {
                error = 2*Math.PI + dif;
            } else {
                error = dif;
            }
        }

        // oscillations occur when chaning direction
        int sign = sign(error);
        if(sign*prevSign == -1)
            oscillations++;
        prevSign = sign;

        double speed = pid.calculate(error);

        // cap the number of ocillations?
        if(pid.onTarget()) {
            // want to exit at a low velocity but don't oscillate forever
            if(overshoot || Math.abs(localizer.getLVel()) < EXIT_SPEED || oscillations > MAX_OSCILLATIONS) {
                abort();
            }
        }
        else {
            robotDrive.tankDrive(-speed, speed);
        }
    }
    
    public void abort() {
        done = true;
        robotDrive.tankDrive(0,0);
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
        pid.setP(Kp);
    }

    public void setKi(double Ki) {
        this.Ki = Ki;
        pid.setI(Ki);
    }

    public void setKd(double Kd) {
        this.Kd = Kd;
        pid.setD(Kd);
    }

    public void setPID(double Kp, double Ki, double Kd) {
        setKp(Kp);
        setKi(Ki);
        setKd(Kd);
    }

}
