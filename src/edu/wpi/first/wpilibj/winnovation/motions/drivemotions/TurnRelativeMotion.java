/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.motions.drivemotions;

import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import edu.wpi.first.wpilibj.winnovation.robot.Localizer;

/**
 *
 * @author Chris
 */
public class TurnRelativeMotion implements Motion {

    private TurnAbsoluteMotion motion;
    private double degrees;
    private boolean hasInitialized = false;
    private double speed;
    private double error;
    private boolean allowOvershoot;

    public TurnRelativeMotion(double degrees, double speed, double error, boolean allowOvershoot) {
        this.degrees = degrees;
        this.speed = speed;
        this.error = error;
        this.allowOvershoot = allowOvershoot;
    }

    public TurnRelativeMotion(double degrees, double speed, double error) {
        this(degrees, speed, error, false);
    }

    public TurnRelativeMotion(double degrees, double speed, boolean allowOvershoot) {
        this(degrees, speed, TurnAbsoluteMotion.DEFAULT_ERROR, allowOvershoot);
    }

    public TurnRelativeMotion(double degrees, double speed) {
        this(degrees, speed, TurnAbsoluteMotion.DEFAULT_ERROR, false);
    }

    private void init() {
        double curHeading = Localizer.getInstance().getTh()*180.0/Math.PI;
        double target = curHeading + degrees;
        motion = new TurnAbsoluteMotion(target, speed, error, allowOvershoot);
        hasInitialized = true;
    }

    public boolean isDone() {
        return motion.isDone();
    }

    public void doMotion() {
        if(!hasInitialized)
            init();
        motion.doMotion();
    }

    public void abort() {
        motion.abort();
    }

}
