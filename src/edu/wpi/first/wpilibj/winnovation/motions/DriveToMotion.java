/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.motions;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.winnovation.robot.Localizer;
import edu.wpi.first.wpilibj.winnovation.robot.PIDRobotDrive;
import edu.wpi.first.wpilibj.winnovation.utils.RealPose2D;

/**
 *
 * @author Chris
 */
public class DriveToMotion implements Motion {

    private boolean done;
    private PIDController controller;

    // PID params
    private double Kp = 1.0;
    private double Ki = 0.0;
    private double Kd = 0.0;


    /**
     * turns to the provided heading
     *
     * @param heading (degrees)
     */
    public DriveToMotion(PIDRobotDrive robotDrive, Localizer localizer, double x, double y) {
        RealPose2D target = new RealPose2D(x,y,0);
        done = false;
        controller = new PIDController(Kp, Ki, Kd, 
                new PIDIn(localizer, target), new PIDOut(robotDrive, localizer, target));
        controller.setContinuous();
        controller.setOutputRange(-1.0, 1.0);
        controller.setSetpoint(0);
    }

    public boolean isDone() {
        return done;
    }

    public void doMotion() {
        if(!controller.isEnable()) {
            controller.enable();
        }
        else if(Math.abs(controller.getError()) < 0.05) {
            done = true;
            controller.disable();
        }
    }

    public void abort() {
        controller.disable();
        done = true;
    }

    private class PIDOut implements PIDOutput {

        private PIDRobotDrive robotDrive;
        private Localizer localizer;
        private RealPose2D target;

        public PIDOut(PIDRobotDrive robotDrive, Localizer localizer, RealPose2D target) {
            this.robotDrive = robotDrive;
            this.localizer = localizer;
            this.target = target;
        }

        public void pidWrite(double output) {

            // transform the target to coordinates with respect to the robot
            // and arc toward this point
            RealPose2D transform = (new RealPose2D(localizer.getX(), localizer.getY(), localizer.getTh())).inverse();
            RealPose2D t = RealPose2D.multiply(transform, target);

            // compute desired curvature
            double x = t.getX();
            double y = t.getY();
            double r = y != 0 ? (x*x + y*y)/(2.0*y) : Double.MAX_VALUE;
            double k = 1/r;

            if(x < 0)
                output *= -1;

            robotDrive.drive(output, k);
        }
    }

    private class PIDIn implements PIDSource {

        private Localizer localizer;
        private RealPose2D target;

        public PIDIn(Localizer localizer, RealPose2D target) {
            this.localizer = localizer;
            this.target = target;
        }

        public double pidGet() {

            // return the distance between the target and current position
            RealPose2D curPose = new RealPose2D(localizer.getX(), localizer.getY(), localizer.getTh());
            return RealPose2D.hypot(curPose, target);

        }

    }

}
