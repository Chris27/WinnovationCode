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

    private final double TOLERANCE = 3.0/12.0; // error tolerance (ft)

    private boolean done;
    private ThreadlessPID pid;

    // PID params
    private double Kp = 0.2;
    private double Ki = 0.0;
    private double Kd = 0.0;
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
        }

        // get current location of target with respect to the robot
        RealPose2D transform = (new RealPose2D(localizer.getX(), localizer.getY(), localizer.getTh())).inverse();
        RealPose2D t = RealPose2D.multiply(transform, target);

        // compute desired curvature
        double x = t.getX();
        double y = t.getY();
        double k = (2*y)/(x*x + y*y);
        double sign = x >= 0 ? 1 : -1;

        // error is distance to the target
        double error = Math.sqrt(x*x + y*y)*sign;
        // comput output speed
        double v = pid.calculate(error);
        
        robotDrive.drive(v-v*k*Constants.WHEEL_BASE_WIDTH/2.0, v+v*k*Constants.WHEEL_BASE_WIDTH/2.0);

        if(pid.onTarget()) {
            abort();
        }

    }

    public void abort() {
        done = true;
        robotDrive.drive(0, 0);

    }

//    private class PIDOut implements PIDOutput {
//
//        private PIDRobotDrive robotDrive;
//        private Localizer localizer;
//        private RealPose2D target;
//
//        public PIDOut(PIDRobotDrive robotDrive, Localizer localizer, RealPose2D target) {
//            this.robotDrive = robotDrive;
//            this.localizer = localizer;
//            this.target = target;
//        }
//
//        public void pidWrite(double output) {
//
//            // transform the target to coordinates with respect to the robot
//            // and arc toward this point
//            RealPose2D transform = (new RealPose2D(localizer.getX(), localizer.getY(), localizer.getTh())).inverse();
//            RealPose2D t = RealPose2D.multiply(transform, target);
//
//            // compute desired curvature
//            double x = t.getX();
//            double y = t.getY();
//            double r = y != 0 ? (x*x + y*y)/(2.0*y) : Double.MAX_VALUE;
//            double k = 1/r;
//
//            if(x < 0)
//                output *= -1;
//
//            robotDrive.drive(output, k);
//        }
//    }
//
//    private class PIDIn implements PIDSource {
//
//        private Localizer localizer;
//        private RealPose2D target;
//
//        public PIDIn(Localizer localizer, RealPose2D target) {
//            this.localizer = localizer;
//            this.target = target;
//        }
//
//        public double pidGet() {
//
//            // return the distance between the target and current position
//            RealPose2D curPose = new RealPose2D(localizer.getX(), localizer.getY(), localizer.getTh());
//            return RealPose2D.hypot(curPose, target);
//
//        }
//
//    }

}
