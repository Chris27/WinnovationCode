/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.motions.auxiliarymotions;

import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import edu.wpi.first.wpilibj.winnovation.robot.AuxiliaryDrive;

/**
 *
 * @author Chris
 */
public class ReleaseTubeMotion implements Motion {

    private AuxiliaryDrive auxiliaryDrive;

    public ReleaseTubeMotion(AuxiliaryDrive auxiliaryDrive) {
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
