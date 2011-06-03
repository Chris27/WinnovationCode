/*
 * Pulse Trigger Boolean
 *
 * This is a boolean that will only return true once per cycle.  This is useful
 * for processing button presses as a button press should only be counted once,
 * not the bajillion times the code will get the button signal in the time it
 * takes to press the button.
 */
package edu.wpi.first.wpilibj.winnovation.utils;

/**
 *
 * @author NUTRONS_PROGRAMMING
 */
public class PulseTriggerBoolean {

    private boolean state = false;
    private boolean oldIn = false;

    public void set(boolean in) {
        if(oldIn == false && in == true) {
            state = true;
        }
        else {
            state = false;
        }
        oldIn = in;
    }

    public boolean get() {
        return state;
    }
}
