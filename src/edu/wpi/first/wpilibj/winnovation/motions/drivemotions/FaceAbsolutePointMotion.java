package edu.wpi.first.wpilibj.winnovation.motions.drivemotions;

import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import edu.wpi.first.wpilibj.winnovation.robot.Localizer;
import edu.wpi.first.wpilibj.winnovation.utils.Angle;
import edu.wpi.first.wpilibj.winnovation.utils.RealPose2D;

/**
 * Turns the robot in place to face towards an (x,y) point in world coordinates
 * (i.e. with respect to the location where the robot started the match)
 * 
 * @author Chris
 */
public class FaceAbsolutePointMotion implements Motion {

    private FaceRelativePointMotion motion;
    private double desiredX;
    private double desiredY;
    private double speed;
    private double error;
    private boolean allowOvershoot;
    private boolean aborted = false;

    /**
     *
     * @param x feet (can't be 0 if y is 0)
     * @param y feet (can't be 0 if x is 0)
     * @param speed speed of rotation [0,1]
     * @param error allowable error (feet)
     * @param allowOvershoot set true if you don't want the robot to oscillate
     */
    public FaceAbsolutePointMotion(double x, double y, double speed, double error, boolean allowOvershoot) {
        this.desiredX = x;
        this.desiredY = y;
        this.speed = speed;
        this.error = error;
        this.allowOvershoot = allowOvershoot;
    }

    public FaceAbsolutePointMotion(double x, double y, double speed, double error) {
        this(x, y, speed, error, false);
    }

    public FaceAbsolutePointMotion(double x, double y, double speed, boolean allowOvershoot) {
        this(x, y, speed, TurnAbsoluteMotion.DEFAULT_ERROR, allowOvershoot);
    }

    public FaceAbsolutePointMotion(double x, double y, double speed) {
        this(x, y, speed, TurnAbsoluteMotion.DEFAULT_ERROR, false);
    }

    private void init() {
        // get the desired x y coordinates relative to the current position of the robot
        Localizer localizer = Localizer.getInstance();
        RealPose2D transform = (new RealPose2D(localizer.getX(), localizer.getY(),
                Angle.normalize(localizer.getTh()))).inverse();
        RealPose2D target = RealPose2D.multiply(transform, new RealPose2D(desiredX, desiredY, 0));
        double xRel = target.getX();
        double yRel = target.getY();
        // this motion has now been reduced to a FaceRelativePointMotion
        motion = new FaceRelativePointMotion(xRel, yRel, speed, error, allowOvershoot);
    }

    public boolean isDone() {
        return aborted || (motion != null && motion.isDone());
    }

    public void doMotion() {
        if (motion == null) {
            init();
        }
        if (isDone()) {
            abort();
        } else {
            motion.doMotion();
        }
    }

    public void abort() {
        if (motion != null) {
            motion.abort();
        }
        aborted = true;
    }
}
