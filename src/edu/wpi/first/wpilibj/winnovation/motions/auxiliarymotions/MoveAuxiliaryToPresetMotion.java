
package edu.wpi.first.wpilibj.winnovation.motions.auxiliarymotions;

import edu.wpi.first.wpilibj.winnovation.motions.Motion;
import edu.wpi.first.wpilibj.winnovation.robot.AuxiliaryDrive;

/**
 * TO BE IMPLEMENTED...
 * 
 * Motion that moves the arm/auxiliary to a preset
 *
 * @author Chris
 */
public class MoveAuxiliaryToPresetMotion implements Motion {

    private AuxiliaryDrive auxiliaryDrive;
    private final int preset;
    private boolean isDone;

    public MoveAuxiliaryToPresetMotion(AuxiliaryDrive auxiliaryDrive, final int preset) {
        this.preset = preset;
        this.auxiliaryDrive = auxiliaryDrive;
    }


    public boolean isDone() {
        return isDone;
    }

    public void doMotion() {
        switch(preset) {
            case AuxiliaryPreset.GROUND:
                // move the arm...
                break;
            case AuxiliaryPreset.HIGHROWMID:
                // move the arm...
                break;
            case AuxiliaryPreset.HIGHROWSIDE:
                // move the arm...
                break;
            case AuxiliaryPreset.MIDROWMID:
                // move the arm...
                break;
            case AuxiliaryPreset.MIDROWSIDE:
                // move the arm...
                break;
            case AuxiliaryPreset.LOWROWMID:
                // move the arm...
                break;
            case AuxiliaryPreset.LOWROWSIDE:
                // move the arm...
                break;
            default:
                // do nothing
                break;
        }
        
    }

    public void abort() {
        isDone = true;
    }

}
