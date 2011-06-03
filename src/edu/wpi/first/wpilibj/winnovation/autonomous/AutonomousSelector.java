
package edu.wpi.first.wpilibj.winnovation.autonomous;

/**
 * NEEDS TO BE IMPLEMENTED
 *
 * This class should set up some DigitalInputs for each of the autonomous mode
 * selector switches on the bot, read them, and output the corresponding mode
 * in the getAutonomousMode() method
 *
 * @author Chris
 */
public class AutonomousSelector {

    // IMPLEMENT ME!!!

    // Should probably have some DigitalInputs's as class memeber variables...

    private static AutonomousSelector instance;

    public AutonomousSelector() {
        // should just initialize a bunch of digital inputs (assuming using
        // switches to choose auto mode
    }

    public static AutonomousSelector getInstance() {
        if(instance == null) {
            instance = new AutonomousSelector(/*...*/);
        }
        return instance;
    }

    public AutonomousMode getAutonomousMode() {
        // should look at the switches and select the appropriate autonomous mode

        return new DoNothing();
    }

}
