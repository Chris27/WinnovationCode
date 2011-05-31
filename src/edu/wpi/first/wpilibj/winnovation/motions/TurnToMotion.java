
package edu.wpi.first.wpilibj.winnovation.motions;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.winnovation.robot.Localizer;
import edu.wpi.first.wpilibj.winnovation.utils.Angle;
import edu.wpi.first.wpilibj.winnovation.utils.PIDTunable;
import edu.wpi.first.wpilibj.winnovation.utils.ThreadlessPID;

/**
 * Turns the robot in place to the desired heading
 *
 * @author Chris
 */
public class TurnToMotion implements Motion, PIDTunable {

    private boolean done;
    private ThreadlessPID pid;
    private RobotDrive robotDrive;
    private Localizer localizer;
    private double target; // in radians


    private final double Threshold = 2.0*Math.PI/180.0; // accept 2 degrees of error
    private final double EXIT_SPEED = 1.3; // ft/s
    private final int MAX_OSCILLATIONS = 5;

    private double Kp = 0.005*55;
    private double Ki = 0.005*2;
    private double Kd = 0.005*7;


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
