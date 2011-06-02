
package edu.wpi.first.wpilibj.winnovation.motions;

/**
 * Motions are primitive actions.  By defining a few simple action, making an
 * autonomous mode should be as simple as providing a list of these motions
 * (see AutonomousMode).  I have defined several drive train motions implementing
 * this interface that should let you drive and position the robot consistently
 *
 * To do: Implement motions for the auxiliary.  Tweak drive motions (PID values).
 *
 * @author Chris
 */
public interface Motion {
    boolean isDone();
    void doMotion();
    void abort();

}
