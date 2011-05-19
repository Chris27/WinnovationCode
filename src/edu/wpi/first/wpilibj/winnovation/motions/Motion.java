/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.motions;

/**
 *
 * @author Chris
 */
public interface Motion {
    boolean isDone();
    void doMotion();
    void abort();

}
