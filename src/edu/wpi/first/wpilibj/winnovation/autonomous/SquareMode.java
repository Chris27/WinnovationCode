/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.autonomous;

import edu.wpi.first.wpilibj.winnovation.motions.drivemotions.DriveToAbsolutePointMotion;
import edu.wpi.first.wpilibj.winnovation.motions.drivemotions.TurnAbsoluteMotion;

/**
 * Just to test
 *
 * @author Chris
 */
public class SquareMode extends AutonomousMode {


    public SquareMode() {
        super();
        driveMotions.addElement(new DriveToAbsolutePointMotion(5.0, 0, 0.5, true));
        driveMotions.addElement(new TurnAbsoluteMotion(90, 0.5));
        driveMotions.addElement(new DriveToAbsolutePointMotion(5.0, 5.0, 0.5, true));
        driveMotions.addElement(new TurnAbsoluteMotion(179, 0.5));
        driveMotions.addElement(new DriveToAbsolutePointMotion(0.0, 5.0, 0.5, true));
        driveMotions.addElement(new TurnAbsoluteMotion(-90, 0.5));
        driveMotions.addElement(new DriveToAbsolutePointMotion(0, 0, 0.5, true));
    }

}
