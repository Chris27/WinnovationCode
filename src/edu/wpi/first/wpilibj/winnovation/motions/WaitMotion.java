
package edu.wpi.first.wpilibj.winnovation.motions;

import edu.wpi.first.wpilibj.Timer;

/**
 * Motion that waits for x seconds
 *
 * @author Chris
 */
public class WaitMotion implements Motion{

    private double startTime = -1;
    private double waitTime;
    private boolean aborted;
    private boolean started = false;

    public WaitMotion(double seconds) {
        aborted = (seconds <= 0);
        waitTime = seconds;
    }

    public boolean isDone() {
        return aborted || (started && Timer.getFPGATimestamp() >= startTime + waitTime);
    }


    public void doMotion() {
        // dont't start the clock until first call to doMotion();
        if(!started) {
            started = true;
            startTime = Timer.getFPGATimestamp();
        }
    }

    public void abort() {
        aborted = true;
    }

}
