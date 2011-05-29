
package edu.wpi.first.wpilibj.winnovation.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.SmartDashboard;
import edu.wpi.first.wpilibj.winnovation.utils.Angle;

/**
 * Keeps track of the position and velocity of the robot
 *
 * @author Chris
 */
public class Localizer {

    private Gyro gyro;
    private Encoder lEncoder;
    private Encoder rEncoder;

    private double x; // ft
    private double y; // ft
    private double th; // rad
    private double lDist; // ft
    private double rDist; // ft
    private double thVel; //rad/sec
    private double lVel;  // ft/s
    private double rVel;  // ft/s
    
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
     * @return heading in radians  (x-axis has heading 0)
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
     * @return angular velocity in radians/second (counter-clockwise is +)
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
    public synchronized void update() {

        long curTime = System.currentTimeMillis();
        double delT = ((double)(curTime - time))/1000.0; // seconds since last update()

        // update current velocities
        lVel = (lEncoder.getDistance() - lDist)/delT; // because encoder getRate() seems to be broken...
        rVel = (rEncoder.getDistance() - rDist)/delT;
        if(Constants.USE_GYRO)
            thVel = (gyro.getAngle()*Math.PI/180.0 - th)/delT; // gyro is off...
        else
            thVel = (rVel - lVel) / (Constants.WHEEL_BASE_WIDTH);




        double w = thVel; // convert to radians as expected by Java trig functions
        double v = (lVel + rVel)/2.0;

        // integrate velocities uses 4th order Runge-Kutta to get position
        double k00 = v*Math.cos(th);
        double k01 = v*Math.sin(th);
        double k02 = w;
        double k10 = v*Math.cos(th + delT/2.0*w);
        double k11 = v*Math.sin(th + delT/2.0*w);
        double k12 = w;
        double k20 = v*Math.cos(th + delT/2.0*w);
        double k21 = v*Math.sin(th + delT/2.0*w);
        double k22 = w;
        double k30 = v*Math.cos(th + delT*w);
        double k31 = v*Math.sin(th + delT*w);
        double k32 = w;

        x = x + delT/6.0*(k00 + 2.0*(k10+k20) + k30);
        y = y + delT/6.0*(k01 + 2.0*(k11+k21) + k31);
        if(Constants.USE_GYRO)
            th = gyro.getAngle();
        else
            th = th + (delT/6.0*(k02 + 2.0*(k12+k22) + k32));

        lDist = lEncoder.getDistance();
        rDist = rEncoder.getDistance();
        time = curTime;

        // log stuff
        SmartDashboard.log(lEncoder.getDistance(), "lEncoder (ft)");
        SmartDashboard.log(rEncoder.getDistance(), "rEncoder (ft)");
        SmartDashboard.log(lVel, "lVel (ft/s)");
        SmartDashboard.log(rVel, "rVel (ft/s)");
        SmartDashboard.log(gyro.getAngle(), "gyro");
        SmartDashboard.log(x, "x (ft)");
        SmartDashboard.log(y, "y (ft)");
        SmartDashboard.log(Angle.normalizeDeg(th*180.0/Math.PI), "heading (deg)");
        SmartDashboard.log(v, "lin speed (ft/s)");
        SmartDashboard.log(w*180.0/Math.PI, "rot speed (deg/s)");
        SmartDashboard.log(delT, "delT");
    }

    public synchronized void reset() {
        lEncoder.reset();
        rEncoder.reset();
        gyro.reset();
        x = 0;
        y = 0;
        th = 0;
        lDist = 0;
        rDist = 0;
        lVel = 0;
        rVel = 0;
        thVel = 0;
    }


}
