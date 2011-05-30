
package edu.wpi.first.wpilibj.winnovation.motions;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SmartDashboard;
import edu.wpi.first.wpilibj.winnovation.robot.Localizer;
import edu.wpi.first.wpilibj.winnovation.utils.Angle;
import edu.wpi.first.wpilibj.winnovation.utils.ThreadlessPID;

/**
 * Turns the robot in place to the desired heading
 *
 * @author Chris
 */
public class TurnToMotion implements Motion {

    private boolean done;
    private ThreadlessPID pid;
    private RobotDrive robotDrive;
    private Localizer localizer;
    private double target; // radians


    private final double Threshold = 2.0*Math.PI/180.0; // accept 2 degrees of error
    private final double EXIT_SPEED = 2.0; // ft/s

    // PID params
    private final double Kp = 0.10;
    private final double Ki = 0.01;
    private final double Kd = 0.01;

    /**
     * Turns to the provided heading
     *
     * @param heading (degrees)
     */
    public TurnToMotion(RobotDrive robotDrive, double speed, Localizer localizer, double heading) {
        done = false;
        pid = new ThreadlessPID(Kp, Ki, Kd);
        pid.setOutputRange(-speed, speed);
        pid.setSetpoint(0);
        pid.setTolerance(Threshold);
        this.robotDrive = robotDrive;
        this.localizer = localizer;
        this.target = Angle.normalize(heading*Math.PI/180.0);

    }

    public boolean isDone() {
        return done;
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

        double speed = pid.calculate(error);

        if(pid.onTarget() && Math.abs(localizer.getLVel()) < EXIT_SPEED) {
            abort();
        }
        else {
            SmartDashboard.log(speed, "rotate at speed");
            robotDrive.tankDrive(-speed, speed);
        }
    }
    
    public void abort() {
        done = true;
        robotDrive.tankDrive(0,0);
    }

}
