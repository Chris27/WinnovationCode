/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.motions;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SmartDashboard;
import edu.wpi.first.wpilibj.winnovation.robot.Constants;
import edu.wpi.first.wpilibj.winnovation.robot.Localizer;
import edu.wpi.first.wpilibj.winnovation.utils.Angle;
import edu.wpi.first.wpilibj.winnovation.utils.ThreadlessPID;

/**
 *
 * @author Chris
 */
public class TurnToMotion implements Motion {

    private boolean done;
    private ThreadlessPID pid;
    private RobotDrive robotDrive;
    private Localizer localizer;
    private double target; // radians

    private final double ErrorThresh = 2.0*Math.PI/180.0; // accept 2 degrees of error

    // PID params
    private final double Kp = 0.6;
    private final double Ki = 0.0;
    private final double Kd = 0.0;


    /**
     * turns to the provided heading
     *
     * @param heading (degrees)
     */
    public TurnToMotion(RobotDrive robotDrive, Localizer localizer, double heading) {
        done = false;
        pid = new ThreadlessPID(Kp, Ki, Kd);
        pid.setContinuous();
        pid.setOutputRange(-1.0, 1.0);
        pid.setInputRange(-Math.PI, Math.PI);
        pid.setSetpoint(0);
        this.robotDrive = robotDrive;
        this.localizer = localizer;
        this.target = Angle.normalize(heading*Math.PI/180.0);
         // not really needed?
    }

    public boolean isDone() {
        return done;
    }

    public void doMotion() {

        double curHeading = Angle.normalize(localizer.getTh());
        //boolean goCC = (target - curHeading) < Math.PI;
        double dif = target - curHeading;
        double error;

        if (dif > 0) {
            if (dif < Math.PI) {
                error = dif;
            } else {
                error = -1 * (2 * Math.PI - dif);
            }
        } else {
            if (Math.abs(dif) > Math.PI) {
                error = 2 * Math.PI + dif;
            } else {
                error = dif;
            }
        }

        double speed = pid.calculate(error);

//        if(Math.abs(pid.getError()) < ErrorThresh) {
//            abort();
//        }
//        else {
            SmartDashboard.log(speed, "rotate at speed");
            robotDrive.tankDrive(-speed, speed);
//        }
    }
    
    public void abort() {
        done = true;
        robotDrive.tankDrive(0,0);
    }

//    private class PIDOut implements PIDOutput {
//
//        private RobotDrive robotDrive;
//
//        public PIDOut(RobotDrive robotDrive) {
//            this.robotDrive = robotDrive;
//        }
//
//        public void pidWrite(double output) {
//            robotDrive.tankDrive(-1*output, output);
//        }
//    }
//
//    private class PIDIn implements PIDSource {
//
//        private Localizer localizer;
//        private double target;
//
//        public PIDIn(Localizer localizer, double target) {
//            this.localizer = localizer;
//            this.target = target;
//        }
//
//        public double pidGet() {
//            return Angle.normalizeDeg(localizer.getTh());
//        }
//
//    }

}
