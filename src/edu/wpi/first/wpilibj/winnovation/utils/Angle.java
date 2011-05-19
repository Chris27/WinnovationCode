/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.winnovation.utils;

import com.sun.squawk.util.MathUtils;

/**
 *
 * @author Chris
 */
public class Angle {

    /**
     * Normalize the angle to the range -PI <= angle < PI.
     * @param angleValue
     */
    public static double normalize(double angleValue) {
        return angleValue = angleValue - 2.0*Math.PI*(MathUtils.round(angleValue/(2.0*Math.PI)));
    }

    public static double normalizeDeg(double angleValue) {
        return angleValue = angleValue - 360.0*(MathUtils.round(angleValue/360.0));
    }
}
