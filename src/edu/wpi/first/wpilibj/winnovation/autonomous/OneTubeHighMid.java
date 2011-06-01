/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.autonomous;

import edu.wpi.first.wpilibj.winnovation.motions.auxiliarymotions.AuxiliaryPreset;
import edu.wpi.first.wpilibj.winnovation.motions.auxiliarymotions.ReleaseTubeMotion;
import edu.wpi.first.wpilibj.winnovation.motions.auxiliarymotions.MoveAuxiliaryToPresetMotion;
import edu.wpi.first.wpilibj.winnovation.motions.drivemotions.DriveStraightMotion;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.winnovation.robot.AuxiliaryDrive;
import edu.wpi.first.wpilibj.winnovation.robot.Localizer;
import edu.wpi.first.wpilibj.winnovation.motions.*;

/**
 *
 * @author Chris
 */
public class OneTubeHighMid extends AutonomousMode {

    private double FORWARD_DISTANCE = 6.0; // feet to the rack
    private double FORWARD_SPEED = 0.4;
    private double BACKUP_DISTANCE = 3.0; //
    private double ARM_DELAY = 1.0;
    private double WRIST_SCORE_ANGLE = 0;

    public OneTubeHighMid(RobotDrive robotDrive, AuxiliaryDrive auxiliaryDrive, Localizer localizer) {
        super();


        /* Drive Motions */
        // drive to the rack
        driveMotions.addElement(new DriveStraightMotion(-FORWARD_DISTANCE, FORWARD_SPEED));
        // wait for the arm to get to position
        driveMotions.addElement(new SynchronizeMotion());
        // wait for the arm to score
        driveMotions.addElement(new SynchronizeMotion());
        // back up
        driveMotions.addElement(new DriveStraightMotion(BACKUP_DISTANCE, FORWARD_SPEED));

        /* Auxiliary motions */
        // drive a bit before bringing the arm up
        auxMotions.addElement(new WaitMotion(ARM_DELAY));
        // bring the arm up to scoring position
        auxMotions.addElement(new MoveAuxiliaryToPresetMotion(auxiliaryDrive, AuxiliaryPreset.HIGHROWMID));
        // wait for drive train to arrive at rack
        auxMotions.addElement(new SynchronizeMotion());
        // rotate wrist to place tube
        auxMotions.addElement(new WristMotion(auxiliaryDrive, WRIST_SCORE_ANGLE));
        // spit out tube
        auxMotions.addElement(new ReleaseTubeMotion(auxiliaryDrive));
        // allow drive train to back up
        auxMotions.addElement(new SynchronizeMotion());
        // drop down to ground preset
        auxMotions.addElement(new MoveAuxiliaryToPresetMotion(auxiliaryDrive, AuxiliaryPreset.GROUND));
        


    }

}
