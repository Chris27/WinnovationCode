/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.motions;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.winnovation.robot.Localizer;
import edu.wpi.first.wpilibj.winnovation.utils.Angle;

/**
 *
 * @author Chris
 */
public class TurnToMotion implements Motion {

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
    public TurnToMotion(RobotDrive robotDrive, Localizer localizer, double heading) {
        done = false;
        controller = new PIDController(Kp, Ki, Kd, new PIDIn(localizer, heading), new PIDOut(robotDrive));
        controller.setContinuous();
        controller.setOutputRange(-1.0, 1.0);
        controller.setInputRange(-180, 180);
        controller.setSetpoint(heading);
        controller.setTolerance(0.1);
    }

    public boolean isDone() {
        return done;
    }

    public void doMotion() {
        if(!controller.isEnable()) {
            controller.enable();
        }
        else if(controller.onTarget()) {
            done = true;
            controller.disable();
        }
    }
    
    public void abort() {
        controller.disable();
        done = true;
    }

    private class PIDOut implements PIDOutput {

        private RobotDrive robotDrive;

        public PIDOut(RobotDrive robotDrive) {
            this.robotDrive = robotDrive;
        }

        public void pidWrite(double output) {
            robotDrive.tankDrive(-1*output, output);
        }
    }

    private class PIDIn implements PIDSource {

        private Localizer localizer;
        private double target;

        public PIDIn(Localizer localizer, double target) {
            this.localizer = localizer;
            this.target = target;
        }

        public double pidGet() {
            return Angle.normalizeDeg(localizer.getTh());
        }

    }

}
