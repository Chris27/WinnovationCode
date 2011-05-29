/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.autonomous;

import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import edu.wpi.first.wpilibj.winnovation.motions.SynchronizeMotion;
import java.util.Vector;

/**
 *
 * @author Chris
 */
public abstract class AutonomousMode {

    protected Vector driveMotions;
    protected Vector auxMotions;
    
    protected AutonomousMode() {
        driveMotions = new Vector();
        auxMotions = new Vector();
    }

    public void run() {
        Motion driveMotion = null;
        Motion auxMotion = null;

        // get and do drive motion
        if(driveMotions.size() > 0) {
            driveMotion = (Motion) driveMotions.firstElement();
            driveMotion.doMotion();
            if(driveMotion.isDone())
                driveMotions.removeElement(driveMotion);
        }
        // get auxilary motion
        if(auxMotions.size() > 0) {
            auxMotion = (Motion) auxMotions.firstElement();
            auxMotion.doMotion();
            if(auxMotion.isDone())
                auxMotions.removeElement(auxMotion);
        }

        // check if queues are waiting to synchronize
        if(driveMotion instanceof SynchronizeMotion && auxMotion instanceof SynchronizeMotion) {
            driveMotions.removeElement(driveMotion);
            driveMotions.removeElement(auxMotion);
        }
    }

}
