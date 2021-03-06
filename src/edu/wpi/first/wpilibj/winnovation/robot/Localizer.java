package edu.wpi.first.wpilibj.winnovation.robot;

import edu.wpi.first.wpilibj.winnovation.config.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.winnovation.utils.Angle;
import edu.wpi.first.wpilibj.winnovation.utils.FixedGyro;

/**
 * Keeps track of the position, orientation, and velocity of the robot.  Must call
 * update() periodically.  If you lobster the drive train, call setLobsters() so
 * it knows about the 90 degree shift in orientation
 *
 * @author Chris
 */
public class Localizer {

    private static Localizer instance = null;
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
    private double time; // seconds

    private boolean lobstered = false;

    public Localizer(Gyro gyro, Encoder lEncoder, Encoder rEncoder) {
        this.gyro = gyro;
        this.lEncoder = lEncoder;
        this.rEncoder = rEncoder;
        time = Timer.getFPGATimestamp();//System.currentTimeMillis();
        x = y = th = lDist = rDist = thVel = lVel = rVel = 0;
    }

    public static synchronized Localizer getInstance() {
        if (instance == null) {
            Gyro gyro = new FixedGyro(Constants.GYRO_CH);
            Encoder lEncoder = new Encoder(Constants.LEFT_DRIVE_ENCODER_A_CH,
                    Constants.LEFT_DRIVE_ENCODER_B_CH, true, Encoder.EncodingType.k1X);
            Encoder rEncoder = new Encoder(Constants.RIGHT_DRIVE_ENCODER_A_CH,
                    Constants.RIGHT_DRIVE_ENCODER_B_CH, true, Encoder.EncodingType.k1X);
            lEncoder.setDistancePerPulse(Constants.LEFT_DRIVE_DIST_PER_PULSE);
            rEncoder.setDistancePerPulse(Constants.RIGHT_DRIVE_DIST_PER_PULSE);
            lEncoder.start();
            rEncoder.start();
            instance = new Localizer(gyro, lEncoder, rEncoder);
        }
        return instance;
    }

    public double getLDist() {
        return lDist;
    }

    public double getRDist() {
        return rDist;
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
        return (lVel + rVel) / 2.0;
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
     * Call this method every time lobsters are deployed/undeployed to
     * update the localizer about the effective 90 degree rotation
     *
     * @param setLobser true if lobsters are down, false otherwise
     */
    public synchronized void setLobsters(boolean setLobser) {
        // the signs here might be backwards
        if(!lobstered && setLobser) {
            lobstered = true;
            th += Math.PI/2.0;
        } else if(lobstered && !setLobser) {
            lobstered = false;
            th -= Math.PI/2.0;
        }
    }

    /**
     * Updates the position of the robot.  This method should be called
     * regularly and frequently for best accuracy
     *
     */
    public synchronized void update() {

        double curTime = Timer.getFPGATimestamp();//System.currentTimeMillis();
        double delT = curTime - time; //((double) (curTime - time)) / 1000.0; // seconds since last update()

        // update current velocities
        lVel = (lEncoder.getDistance() - lDist) / delT; // because encoder getRate() seems to be broken...
        rVel = (rEncoder.getDistance() - rDist) / delT;
        if (Constants.USE_GYRO) {
            thVel = (gyro.getAngle() * Math.PI / 180.0 - th) / delT; // gyro is off...
        } else {
            thVel = (rVel - lVel) / (Constants.WHEEL_BASE_WIDTH);
        }

        double w = thVel; // convert to radians as expected by Java trig functions
        double v = (lVel + rVel) / 2.0;

        // integrate velocities uses 4th order Runge-Kutta to get position
        double k00 = v * Math.cos(th);
        double k01 = v * Math.sin(th);
        double k02 = w;
        double k10 = v * Math.cos(th + delT / 2.0 * w);
        double k11 = v * Math.sin(th + delT / 2.0 * w);
        double k12 = w;
        double k20 = v * Math.cos(th + delT / 2.0 * w);
        double k21 = v * Math.sin(th + delT / 2.0 * w);
        double k22 = w;
        double k30 = v * Math.cos(th + delT * w);
        double k31 = v * Math.sin(th + delT * w);
        double k32 = w;

        x = x + delT / 6.0 * (k00 + 2.0 * (k10 + k20) + k30);
        y = y + delT / 6.0 * (k01 + 2.0 * (k11 + k21) + k31);
        if (Constants.USE_GYRO) {
            th = gyro.getAngle() * Math.PI / 180.0;
        } else {
            th = th + (delT / 6.0 * (k02 + 2.0 * (k12 + k22) + k32));
        }

        lDist = lEncoder.getDistance();
        rDist = rEncoder.getDistance();
        time = curTime;

        // log stuff
        if (Constants.LOGGING_ENABLED) {
            SmartDashboard.log(lEncoder.getDistance(), "lEncoder (ft)");
            SmartDashboard.log(rEncoder.getDistance(), "rEncoder (ft)");
            SmartDashboard.log(lVel, "lVel (ft/s)");
            SmartDashboard.log(rVel, "rVel (ft/s)");
            SmartDashboard.log(gyro.getAngle(), "gyro");
            SmartDashboard.log(x, "x (ft)");
            SmartDashboard.log(y, "y (ft)");
            SmartDashboard.log(Angle.normalizeDeg(th * 180.0 / Math.PI), "heading (degrees)");
            //SmartDashboard.log(v, "lin speed (ft/s)");
            //SmartDashboard.log(w*180.0/Math.PI, "rot speed (deg/s)");
            //SmartDashboard.log(delT, "delT");
        }
    }

    public synchronized void reset() {
//        lEncoder.reset();
//        rEncoder.reset();
//        gyro.reset();
//        x = 0;
//        y = 0;
//        th = 0;
//        lDist = 0;
//        rDist = 0;
//        lVel = 0;
//        rVel = 0;
//        thVel = 0;
    }
}
