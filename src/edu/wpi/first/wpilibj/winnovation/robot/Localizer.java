
package edu.wpi.first.wpilibj.winnovation.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.SmartDashboard;

/**
 *
 * @author Chris
 */
public class Localizer {

    private Gyro gyro;
    private Encoder lEncoder;
    private Encoder rEncoder;

    private double x;
    private double y;
    private double th;
    private double lDist;
    private double rDist;
    private double thVel;
    private double lVel;
    private double rVel;
    
    private long time;


    public Localizer(Gyro gyro, Encoder lEncoder, Encoder rEncoder) {
        this.gyro = gyro;
        this.lEncoder = lEncoder;
        this.rEncoder = rEncoder;
        time = System.currentTimeMillis();
        x = y = th = lDist = rDist = thVel = lVel = rVel = 0;
    }


    /**
     * The x axis is defined as the forward direction when this Localizer
     * instance was constructed
     *
     * @return x position in feet (center of robot)
     */
    public double getX() {
       return x;
    }

    /**
     * The y axis is defined as the left direction when this Localizer
     * instance was constructed
     *
     * @return y position in feet
     */
    public double getY() {
        return y;
    }

    /**
     * @return heading in degrees [0,360)  (x-axis has heading 0)
     */
    public double getTh() {

        return th;
    }

    /**
     * @return velocity (feet/second)
     */
    public double getVel() {
        return (lVel + rVel)/2.0;
    }

    /**
     * @return velocity of left wheels
     */
    public double getLVel() {
        return lVel;
    }

    /**
     * @return velocity of right wheels
     */
    public double getRVel() {
        return rVel;
    }


    /**
     * @return angular velocity in degrees/second (counter-clockwise is +)
     */
    public double getThVel() {
        return thVel;
    }

    /**
     * Updates the position of the robot.  This method should be called
     * regularly and frequently for best accuracy
     *
     * @param isLobstered
     */
    public void update(/*boolean isLobstered*/) {

        /*
        // lobstering adds a 90 degree rotation
        double offset = isLobstered ? Math.PI/2.0 : 0;*/

        long curTime = System.currentTimeMillis();
        double delT = (curTime - time)/1000; // seconds since last update()

        // update current velocities
        lVel = (lEncoder.getDistance() - lDist)/delT;
        rVel = (rEncoder.getDistance() - rDist)/delT;
        thVel = (gyro.getAngle() - th)/delT;
        double w = thVel*(Math.PI/180.0); // convert to radians as expected by Java trig functions
        double thRad = th*(Math.PI/180.0);
        double v = (lVel + rVel)/2.0;

        // integrate velocities uses 4th order Runge-Kutta to get position
        double k00 = v*Math.cos(thRad);
        double k01 = v*Math.sin(thRad);
        double k10 = v*Math.cos(thRad + delT/2.0*w);
        double k11 = v*Math.sin(thRad + delT/2.0*w);
        double k20 = v*Math.cos(thRad + delT/2.0*w);
        double k21 = v*Math.sin(thRad + delT/2.0*w);
        double k30 = v*Math.cos(thRad + delT*w);
        double k31 = v*Math.sin(thRad + delT*w);

        x = x + delT/6.0*(k00 + 2.0*(k10+k20) + k30);
        y = y + delT/6.0*(k01 + 2.0*(k11+k21) + k31);
        th = gyro.getAngle();

        lDist = lEncoder.getDistance();
        rDist = rEncoder.getDistance();
        time = curTime;

        // log stuff
        SmartDashboard.log(lEncoder.getDistance(), "lEncoder");
        SmartDashboard.log(rEncoder.getDistance(), "rEncoder");
        SmartDashboard.log(gyro.getAngle(), "gyro");
        SmartDashboard.log(x, "x (ft)");
        SmartDashboard.log(y, "y (ft)");
        SmartDashboard.log(th, "heading (deg)");
        SmartDashboard.log(v, "lin speed (ft/s)");
        SmartDashboard.log(w*180.0/Math.PI, "rot speed (deg/s)");
    }


}
