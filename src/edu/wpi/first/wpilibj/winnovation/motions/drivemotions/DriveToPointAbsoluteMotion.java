/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.motions.drivemotions;

import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import edu.wpi.first.wpilibj.winnovation.robot.Localizer;
import edu.wpi.first.wpilibj.winnovation.utils.RealPose2D;

/**
 *
 * @author Chris
 */
public class DriveToPointAbsoluteMotion implements Motion {

    private DriveToPointRelativeMotion motion;

    public DriveToPointAbsoluteMotion(double x, double y, double speed, double error, boolean allowOvershoot) {
        // transform these absolute coordinates to relative coordinates
        Localizer localizer = Localizer.getInstance();
        RealPose2D transform = (new RealPose2D(localizer.getX(), localizer.getY(), localizer.getTh())).inverse();
        RealPose2D pose = RealPose2D.multiply(transform, new RealPose2D(x, y, 0));
        double xRel = pose.getX();
        double yRel = pose.getY();
        motion = new DriveToPointRelativeMotion(xRel, yRel, speed, error, allowOvershoot);
    }

    public DriveToPointAbsoluteMotion(double x, double y, double speed, double error) {
        this(x, y, speed, error, false);
    }
    public DriveToPointAbsoluteMotion(double x, double y, double speed, boolean allowOvershoot) {
        this(x, y, speed, DriveToPointRelativeMotion.DEFAULT_ALLOWABLE_ERROR, allowOvershoot);
    }
    public DriveToPointAbsoluteMotion(double x, double y, double speed) {
        this(x, y, speed, DriveToPointRelativeMotion.DEFAULT_ALLOWABLE_ERROR, false);
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
