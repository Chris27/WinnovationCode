
package edu.wpi.first.wpilibj.winnovation.motions.drivemotions;

import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import edu.wpi.first.wpilibj.winnovation.robot.Localizer;
import edu.wpi.first.wpilibj.winnovation.utils.Angle;
import edu.wpi.first.wpilibj.winnovation.utils.RealPose2D;

/**
 * This motion will arc the robot to the coordinates (x,y).
 * These coordinates are relative to the starting position of the robot.
 * The front of the robot at the start points in the x axis and the left of the robot
 * points in the y axis.  PID control will attempt to stop the robot at the correct
 * distance as well as enforce the right curvature of the trajectory.
 *
 *
 * @author Chris
 */
public class DriveToAbsolutePointMotion implements Motion {

    private DriveToRelativePointMotion motion;

    /**
     *
     * @param x desired x coordinate (feet)
     * @param y desired y coordinate (feet)
     * @param speed ([0,1]) max speed
     * @param error allowed error from the target (feet)
     * @param allowOvershoot true will keep the robot from oscillating around the
     *  target but will let the robot overshoot past the target
     */
    public DriveToAbsolutePointMotion(double x, double y, double speed, double error, boolean allowOvershoot) {
        // transform these absolute coordinates to relative coordinates to convert to a DriveToRelativePointMotion
        Localizer localizer = Localizer.getInstance();
        RealPose2D transform = (new RealPose2D(localizer.getX(), localizer.getY(), Angle.normalize(localizer.getTh()))).inverse();
        RealPose2D pose = RealPose2D.multiply(transform, new RealPose2D(x, y, 0));
        double xRel = pose.getX();
        double yRel = pose.getY();
        motion = new DriveToRelativePointMotion(xRel, yRel, speed, error, allowOvershoot);
    }

    public DriveToAbsolutePointMotion(double x, double y, double speed, double error) {
        this(x, y, speed, error, false);
    }

    public DriveToAbsolutePointMotion(double x, double y, double speed, boolean allowOvershoot) {
        this(x, y, speed, DriveToRelativePointMotion.DEFAULT_ALLOWABLE_ERROR, allowOvershoot);
    }
    public DriveToAbsolutePointMotion(double x, double y, double speed) {
        this(x, y, speed, DriveToRelativePointMotion.DEFAULT_ALLOWABLE_ERROR, false);
    }



    public boolean isDone() {
        return motion.isDone();
    }

    public void doMotion() {
        motion.doMotion();
    }

    public void abort() {
        motion.abort();
    }

}
