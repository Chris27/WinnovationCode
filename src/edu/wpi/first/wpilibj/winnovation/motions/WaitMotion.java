
package edu.wpi.first.wpilibj.winnovation.motions;

/**
 * Wait for x seconds
 *
 * @author Chris
 */
public class WaitMotion implements Motion{

    private long startTime = -1;
    private long waitTime;
    private boolean aborted;

    public WaitMotion(double seconds) {
        aborted = (seconds <= 0);
        waitTime = (long) (1000*seconds);
    }

    public boolean isDone() {
        return aborted || (startTime >= 0 && System.currentTimeMillis() >= startTime + waitTime);
    }


    public void doMotion() {
        // dont't start the clock until first call to doMotion();
        if(startTime < 0)
            startTime = System.currentTimeMillis();
    }

    public void abort() {
        aborted = true;
    }

}
