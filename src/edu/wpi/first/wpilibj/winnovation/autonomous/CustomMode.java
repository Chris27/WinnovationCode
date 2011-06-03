
package edu.wpi.first.wpilibj.winnovation.autonomous;

import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import java.util.Vector;

/**
 * This will construct an AutonomousMode from two lists of motions (drive + arm)
 *
 * @author Chris
 */
public class CustomMode extends AutonomousMode {

    public CustomMode(Vector driveMotions, Vector auxMotions) {

       // check that all motions are valid
        int len = driveMotions.size();
        for(int i = 0; i < len; i++) {
            if(!(driveMotions.elementAt(i) instanceof Motion))
                throw new IllegalArgumentException(driveMotions.elementAt(i) + " is not a valid motion!");
        }
        len = auxMotions.size();
        for(int i = 0; i < len; i++) {
            if(!(auxMotions.elementAt(i) instanceof Motion))
                throw new IllegalArgumentException(auxMotions.elementAt(i) + " is not a valid motion!");
        }

        super.driveMotions = driveMotions;
        super.auxMotions = auxMotions;
    }

}
