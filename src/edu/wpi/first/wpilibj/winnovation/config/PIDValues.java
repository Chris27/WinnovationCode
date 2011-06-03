
package edu.wpi.first.wpilibj.winnovation.config;

/**
 * Store gains for PID controllers here
 *
 * A PID controller helps to quickly reduce error in a system.  It takes the error
 * from a desired target as input and gives back a (motor power) output
 *
 * P (Proportional) contributes a power proportional to the error.  A high P value
 * will make the robot/mechanism move towards the target more quickly but a P that
 * is too high will cause the robot/mechanism to overshoot the target and oscillate
 *
 * I (Integral) contributes a power proportional to the integral of the error.  If
 * error remains but the robot is too close to the target for the P term to have much
 * effect, then the I term will take over, ramp up the power, and insure the robot keeps
 * trying to reach the target.  An I term that is too high may cause the robot to
 * have erratic bursts of power.
 *
 * D (derivative) contributes power proportional to the derivative of the error.
 * The D term is used to dampen the system as it does not contribute much as the
 * robot/mechanism is slowing down (as it approaches the target)
 *
 * Getting the right P,I, and D values is a matter of trial and error. Typically
 * a good strategy is to raise P until the robot/mechanism starts oscillating, cut
 * back on P and raise D such that the robot/mechanism approaches the target quickly
 * but with minimal oscillations, and then raise I to get rid of steady state error.
 *
 * The P,I, and D terms are unfortunately not independent and it can take a lot
 * of playing around with the values to get the controller to behave well.  The
 * PIDTunable class is helpful in letting you adjust PID values on the fly with
 * a game pad.
 * 
 * @author Chris
 */
public class PIDValues {
    
    // for heading correction when driving on a trajectory
    public static double HEADING_CORRECT_P = 0.05 * 185;
    public static double HEADING_CORRECT_I = 0.005 * 10;
    public static double HEADING_CORRECT_D = 0.05 * 15;
    public static double HEADING_CORRECT_CAP = 0.3;

    // for wheel velocities
    public static double VEL_CORRECT_P = 0.070 * 25;
    public static double VEL_CORRECT_I = 0.005 * 29;
    public static double VEL_CORRECT_D = 0.016 * 20;
    public static double VEL_CORRECT_CAP = 5.0; // note that the cap is scaled by the desired velocity

    // for in place rotations
    public static double SPIN_CORRECT_P = 0.005*55;;
    public static double SPIN_CORRECT_I = 0.005*2;
    public static double SPIN_CORRECT_D = 0.005*7;

    // for driving distances
    public static double DIST_CORRECT_P = 0.005 * 130;
    public static double DIST_CORRECT_I = 0.005 * 3;
    public static double DIST_CORRECT_D = 0.005 * 35;


}
