
package edu.wpi.first.wpilibj.winnovation.autonomous;

import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import edu.wpi.first.wpilibj.winnovation.motions.SynchronizeMotion;
import java.util.Vector;

/**
 * This abstract class contains the autonomous routine of processing motions
 * in the driveMotions and auxMotions vectors (lists).  To create an autonomous
 * mode, extend this class and add the motions you want to the lists in the 
 * constructor
 * 
 * @author Chris
 */
public abstract class AutonomousMode {

    protected Vector driveMotions; // list of drive train motions
    protected Vector auxMotions; // list of auxiliary motions
    
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
