

package edu.wpi.first.wpilibj.winnovation.motions;

import edu.wpi.first.wpilibj.winnovation.robot.AuxiliaryDrive;

/**
 * TO BE IMPLEMENTED...
 *
 * Motion that moves the wrist to some absolute position
 *
 * @author Chris
 */
public class WristMotion implements Motion {

    private double angle;
    private AuxiliaryDrive auxiliaryDrive;
    
    public WristMotion(AuxiliaryDrive auxiliaryDrive, double angle) {
        this.angle = angle;
        this.auxiliaryDrive = auxiliaryDrive;
    }

    public boolean isDone() {
        return false;
    }

    public void doMotion() {
    }

    public void abort() {
    }

}
