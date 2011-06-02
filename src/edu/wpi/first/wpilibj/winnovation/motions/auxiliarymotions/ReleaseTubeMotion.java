
package edu.wpi.first.wpilibj.winnovation.motions.auxiliarymotions;

import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import edu.wpi.first.wpilibj.winnovation.motions.WaitMotion;
import edu.wpi.first.wpilibj.winnovation.robot.AuxiliaryDrive;

/**
 * Motion that spits out the tube
 *
 * @author Chris
 */
public class ReleaseTubeMotion implements Motion {

    private AuxiliaryDrive auxiliaryDrive;
    private double runTime = 0.75; // seconds to run the rollers
    private WaitMotion timer;

    public ReleaseTubeMotion(AuxiliaryDrive auxiliaryDrive) {
        this.auxiliaryDrive = auxiliaryDrive;
        this.timer = new WaitMotion(runTime);
    }

    public boolean isDone() {
        return timer.isDone();
    }

    public void doMotion() {
        if(isDone()) {
            abort();
            return;
        }

        // run the rollers
    }

    public void abort() {
        // stop rollers
    }

}
