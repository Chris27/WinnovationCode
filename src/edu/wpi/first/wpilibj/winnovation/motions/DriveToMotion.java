/*
 * moves the robot to an (x,y) position
 */

package edu.wpi.first.wpilibj.winnovation.motions;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.winnovation.robot.Constants;
import edu.wpi.first.wpilibj.winnovation.robot.Localizer;
import edu.wpi.first.wpilibj.winnovation.utils.RealPose2D;
import edu.wpi.first.wpilibj.winnovation.utils.ThreadlessPID;

/**
 *
 * @author Chris
 */
public class DriveToMotion implements Motion {

    private final double TOLERANCE = 4.0/12.0; // error tolerance (ft)
    private final double EXIT_SPEED = 2.0; // ft/s

    private boolean done;
    private ThreadlessPID pid;

    // PID params
    private double Kp = 0.15;
    private double Ki = 0.05;
    private double Kd = 0.04;
    private RobotDrive robotDrive;
    private Localizer localizer;
    private RealPose2D target;

    /**
     * turns to the provided heading
     *
     * @param heading (degrees)
     */
    public DriveToMotion(RobotDrive robotDrive, double power, Localizer localizer, double x, double y) {

        this.target = new RealPose2D(x,y,0);
        this.done = false;
        this.robotDrive = robotDrive;
        this.localizer = localizer;

        pid = new ThreadlessPID(Kp, Ki, Kd);
        pid.setOutputRange(-power, power);
        pid.setSetpoint(0);
        pid.setTolerance(TOLERANCE);

    }

    public boolean isDone() {
        return done;
    }

    public void doMotion() {

        if(done) {
            abort();
            return;
        }

        // get current location of target with respect to the robot
        RealPose2D transform = (new RealPose2D(localizer.getX(), localizer.getY(), localizer.getTh())).inverse();
        RealPose2D t = RealPose2D.multiply(transform, target);

        
        double x = t.getX();
        double y = t.getY();
        
        // if no error, done
        if(x*x+y*y == 0) {
            abort();
            return;
        }

        // compute desired curvature of path to the point
        double k = (2*y)/(x*x + y*y);
        double sign = x >= 0 ? 1 : -1;

        // error is distance to the target
        //double error = Math.sqrt(x*x + y*y)*sign;
        // Just use x for the error so bot doesn't death spin when adjacent to the target
        double error = x;

        // comput output speed
        double v = pid.calculate(error);

        // arc to the desired point
        robotDrive.tankDrive(v-v*k*Constants.WHEEL_BASE_WIDTH/2.0, v+v*k*Constants.WHEEL_BASE_WIDTH/2.0);

        if(pid.onTarget() && Math.abs(localizer.getVel()) < EXIT_SPEED) {
            abort();
        }

    }

    public void abort() {
        done = true;
        robotDrive.tankDrive(0, 0);

    }


}
