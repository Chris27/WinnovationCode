
package edu.wpi.first.wpilibj.winnovation.motions.drivemotions;

import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import edu.wpi.first.wpilibj.winnovation.motions.WaitMotion;
import edu.wpi.first.wpilibj.winnovation.config.Constants;
import edu.wpi.first.wpilibj.winnovation.config.PIDValues;
import edu.wpi.first.wpilibj.winnovation.robot.Localizer;
import edu.wpi.first.wpilibj.winnovation.robot.PIDRobotDrive;
import edu.wpi.first.wpilibj.winnovation.utils.Angle;
import edu.wpi.first.wpilibj.winnovation.utils.PIDTunable;
import edu.wpi.first.wpilibj.winnovation.utils.ThreadlessPID;

/**
 * This motion will drive the robot along a trajectory (given by curvature i.e.
 * 1/radius of the arc) at a given speed for a given time.  PID control will
 * monitor the trajectory
 *
 * @author Chris
 */
public class TimedArcMotion implements Motion, PIDTunable {

    // PID to enforce trajectory
    private double Kp = PIDValues.HEADING_CORRECT_P; //0.05 * 185;
    private double Ki = PIDValues.HEADING_CORRECT_I; //0.005 * 10;
    private double Kd = PIDValues.HEADING_CORRECT_D; //0.05 * 15;
    private double rotCap = PIDValues.HEADING_CORRECT_CAP; //0.3;

    private ThreadlessPID pid;

    private WaitMotion wait;
    private PIDRobotDrive robotDrive;
    private Localizer localizer;

    private double k;
    private double v;

    private double initLDist;
    private double initRDist;
    private double initHeading;

    private boolean hasInitialized = false;


    /**
     *
     * @param curvature 1/curvature is the radius of the arc of the trajectory
     * @param time time to drive (seconds)
     * @param vel velocity [-1,1]
     */
    public TimedArcMotion(double curvature, double time, double vel) {
        wait = new WaitMotion(time);
        pid = new ThreadlessPID(Kp, Ki, Kd);
        pid.setOutputRange(-rotCap, rotCap);
        robotDrive = PIDRobotDrive.getInstance();
        localizer = Localizer.getInstance();
        k = curvature;
        v = vel;
    }

    private void init() {
        initLDist = localizer.getLDist();
        initRDist = localizer.getRDist();
        initHeading = localizer.getTh();
        hasInitialized = true;
        wait.doMotion(); // need to start the timer
    }

    public boolean isDone() {
        return wait.isDone();
    }


    // don't want to grab the current position until it is time for this motion to start
    private double getDesiredHeading(double dist) {
        if(k == 0) {
            return initHeading;
        } else {
            return dist*k + initHeading;
        }
    }

    public void doMotion() {

        // get starting values when starting this motion
        if(!hasInitialized) {
            init();
        }
        if(wait.isDone()) {
            abort();
            return;
        }

        // get distance driven so far
        double dist = ((localizer.getLDist() - initLDist) + (localizer.getRDist() - initRDist)) / 2.0;
        // output velocities to drive along the proper curvature
        double leftSpeed = v - v*k*Constants.WHEEL_BASE_WIDTH/2.0;
        double rightSpeed = v + v*k*Constants.WHEEL_BASE_WIDTH/2.0;

        // calculate what the heading should be and tweak accordingly
        double headingError = Angle.normalize(getDesiredHeading(dist) - localizer.getTh());
        double correction = pid.calculate(headingError);
        if (localizer.getVel() >= 0) {
            leftSpeed -= correction;
        } else {
            rightSpeed += correction;
        }

        // set output speeds
        robotDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void abort() {
        robotDrive.tankDrive(0, 0);
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
