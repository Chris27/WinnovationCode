/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.motions.drivemotions;

import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import edu.wpi.first.wpilibj.winnovation.utils.Trig;

/**
 * This will rotate the robot in place to face an (x,y) point relative
 * to the robot (think polar coordinates).  Probably not useful in itself
 * as it will likely be more intuitive to just use a TurnRelativeMotion but
 * this class is used to implement FaceAbsolutePointMotion
 *
 *
 * @author Chris
 */
public class FaceRelativePointMotion implements Motion {

    private TurnRelativeMotion motion;

    /**
     *
     * @param x feet (can't be 0 if y is 0)
     * @param y feet (can't be 0 if x is 0)
     * @param speed speed of rotation [0,1]
     * @param error allowable error (feet)
     * @param allowOvershoot set true if you don't want the robot to oscillate
     */
    public FaceRelativePointMotion(double x, double y, double speed, double error, boolean allowOvershoot) {
        double rad = Trig.atan2(y, x);
        double deg = rad*180.0/Math.PI;
        motion = new TurnRelativeMotion(deg, speed, error, allowOvershoot);
    }

    public FaceRelativePointMotion(double x, double y, double speed, double error) {
        this(x, y, speed, error, false);
    }


    public FaceRelativePointMotion(double x, double y, double speed, boolean allowOvershoot) {
        this(x, y, speed, TurnAbsoluteMotion.DEFAULT_ERROR, allowOvershoot);
    }

    public FaceRelativePointMotion(double x, double y,  double speed) {
        this(x, y, speed, TurnAbsoluteMotion.DEFAULT_ERROR, false);
    }

    public boolean isDone() {
        return motion.isDone();
    }

    public void doMotion() {
        motion.doMotion();
    }

    public void abort() {
        motion.abort();;
    }



}
