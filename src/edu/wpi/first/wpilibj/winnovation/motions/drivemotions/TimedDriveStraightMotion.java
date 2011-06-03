
package edu.wpi.first.wpilibj.winnovation.motions.drivemotions;

/**
 * Motion that drives straight (forward or backward depending on the sign of vel)
 * for the given time.  PID control will attempt to keep the robot driving straight.
 * This motion is just an instance of TimedArcMotion with a curvature of 0.
 *
 * @author Chris
 */
public class TimedDriveStraightMotion extends TimedArcMotion {


    public TimedDriveStraightMotion(double time, double vel) {
        super(0, time, vel);
    }

}
