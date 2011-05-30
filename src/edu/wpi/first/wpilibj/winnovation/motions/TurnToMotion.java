
package edu.wpi.first.wpilibj.winnovation.motions;

import edu.wpi.first.wpilibj.RobotDrive;
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
    private double target; // in radians


    private final double Threshold = 2.0*Math.PI/180.0; // accept 2 degrees of error
    private final double EXIT_SPEED = 2.0; // ft/s
    private final int MAX_OSCILLATIONS = 1;

    // PID params
    private final double Kp = 0.05;
    private final double Ki = 0.01;
    private final double Kd = 0.01;


    private int oscillations = 0;
    private int prevSign = 0;

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
            if(Math.abs(localizer.getLVel()) < EXIT_SPEED || oscillations > MAX_OSCILLATIONS) {
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

}
