/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.robot;

/**
 * IMPLEMENT ME!
 *
 * Moves the arm/auxiliary system
 *
 * @author Chris
 */
public class AuxiliaryDrive {

    private static AuxiliaryDrive instance = null;


    public AuxiliaryDrive(/*parameters go here*/) {
        // todo
    }

    public static synchronized AuxiliaryDrive getInstance() {
        if(instance == null) {
            // ...
            instance = new AuxiliaryDrive(/*...*/);
        }
        return instance;
    }

    // IMPLEMENT ME!
}
