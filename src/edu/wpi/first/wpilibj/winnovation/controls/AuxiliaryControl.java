
package edu.wpi.first.wpilibj.winnovation.controls;

/**
 * TO BE IMPLEMENTED
 *
 * This should read in the controls of the auxiliary driver and move the arm/auxiliary
 * mechanisms appropriately in the handle() method.  See DriverControl.
 *
 * @author Chris
 */
public class AuxiliaryControl implements Control {

    private static AuxiliaryControl instance;




    public AuxiliaryControl(/*...*/) {
        // todo
    }

    public void handle() {
        // todo
    }

    public static AuxiliaryControl getInstance() {
        if(instance == null) {
            instance = new AuxiliaryControl();
        }
        return instance;
    }

}
