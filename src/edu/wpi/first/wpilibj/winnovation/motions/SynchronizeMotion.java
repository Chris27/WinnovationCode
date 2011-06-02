
package edu.wpi.first.wpilibj.winnovation.motions;

/**
 * To be used to synchronize drive train and auxiliary motions.  There should
 * be an equal number of SynchronizeMotions in each of the driveMotion and
 * auxiliaryMotion lists in an autonomous mode.  Each list be simultaneously
 * waiting on a SynchronizeMotion in order to clear it.  This motion is useful
 * for creating checkpoints e.g. a SynchronizeMotion could occur after the robot
 * has driven to a peg and the arm has deployed to a preset so the robot is only
 * allowed to move on to scoring the tube when both of these sub tasks have been
 * accomplished.
 *
 * @author Chris
 */

public class SynchronizeMotion implements Motion {

    public boolean isDone() {
        return false;
    }

    public void doMotion() {
    }

    public void abort() {
    }

}
