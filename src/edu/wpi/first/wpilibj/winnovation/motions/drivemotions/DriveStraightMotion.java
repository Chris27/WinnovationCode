
package edu.wpi.first.wpilibj.winnovation.motions.drivemotions;


/**
 * This is just an DriveToRelativePointMotion with a 0 y component
 * This will drive forward or backward (depending on the sign of dist) dist
 * feet.  PID will attempt to keep the robot straight and stop the robot at
 * the right distance.
 *
 * @author Chris
 */
public class DriveStraightMotion extends DriveToRelativePointMotion {

    /**
     *
     * @param dist (feet to drive)
     * @param speed [-1,1]
     */
    public DriveStraightMotion(double dist, double speed) {
        super(dist, 0, speed);
    }

}
