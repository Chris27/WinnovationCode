
package edu.wpi.first.wpilibj.winnovation.utils;

import edu.wpi.first.wpilibj.Gyro;

/**
 * corrects gyro position to make the counter clockwise direction positive so
 * gyro measurements correctly adhere to the right hand rule
 * 
 * @author Chris
 */
public class FixedGyro extends Gyro {

    private final double SCALE_FACTOR = 90.0/75.6;  // to make 1 deg actually 1 deg

    public FixedGyro(int slot, int channel) {
        super(slot, channel);
    }
    public FixedGyro(int channel) {
        super(channel);
    }

    public double getAngle() {
        return -super.getAngle()*SCALE_FACTOR;
    }
    public double pidGet() {
        return -super.pidGet()*SCALE_FACTOR;
    }

}
