

package edu.wpi.first.wpilibj.winnovation.utils;

/**
 * Classes that follow this interface can have their PID controls tuned with
 * a PIDTuner
 * 
 * @author Chris
 */
public interface PIDTunable {

    public double getKp();
    public double getKi();
    public double getKd();
    public void setKp(double Kp);
    public void setKi(double Ki);
    public void setKd(double Kd);
    public void setPID(double Kp, double Ki, double Kd);

}
