
package edu.wpi.first.wpilibj.winnovation.motions;

/**
 * Motions can be linked together to form autonomous modes
 *
 * @author Chris
 */
public interface Motion {
    boolean isDone();
    void doMotion();
    void abort();

}
