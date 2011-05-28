package edu.wpi.first.wpilibj.winnovation.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
/**
 *
 * uses PID to enforce actual robot velocity
 *
 * @author Chris
 */
public class PIDRobotDrive extends RobotDrive {

    private static final boolean LEFT = true;
    private static final boolean RIGHT = false;

    private PIDController lController;
    private PIDController rController;
    private Wheels leftWheels;
    private Wheels rightWheels;
    

    // PID constants
    private double Kp = 1.0;
    private double Ki = 0.0;
    private double Kd = 0.0;


    public PIDRobotDrive(Encoder lEncoder, Encoder rEncoder,
            SpeedController lCim1, SpeedController lCim2, SpeedController rCim1,
            SpeedController rCim2) {

        super(lCim1, lCim2, rCim1, rCim2);
        leftWheels = new Wheels(lEncoder, LEFT, lCim1, lCim2);
        rightWheels = new Wheels(rEncoder, RIGHT, rCim1, rCim2);

        lController = new PIDController(Kp, Ki, Kd, leftWheels, leftWheels);
        rController = new PIDController(Kp, Ki, Kd, rightWheels, rightWheels);

        lController.setInputRange(-Constants.MaxDriveSpeed, Constants.MaxDriveSpeed);
        lController.setOutputRange(-1.0, 1.0);
        rController.setInputRange(-Constants.MaxDriveSpeed, Constants.MaxDriveSpeed);
        rController.setOutputRange(-1.0, 1.0);
        lController.setSetpoint(0);
        rController.setSetpoint(0);

        lController.enable();
        rController.enable();

       
        
    }

    public void tankDrive(GenericHID leftStick, GenericHID rightStick) {
        double leftVel = -leftStick.getY()*Constants.MaxDriveSpeed;
        double rightVel = -rightStick.getY()*Constants.MaxDriveSpeed;

        // deadzone
        if(Math.abs(leftVel) < Constants.MinDriveSpeed)
            leftVel = 0;
        if(Math.abs(rightVel) < Constants.MinDriveSpeed)
            rightVel = 0;

        this.tankDrive(leftVel, rightVel);
    }

    public void tankDrive(double leftVel, double rightVel) {
        lController.setSetpoint(leftVel);
        rController.setSetpoint(rightVel);
    }

    public void drive(double vel, double curve) {
        tankDrive(vel - curve*vel*Constants.WheelBaseWidth/2.0, vel + curve*vel*Constants.WheelBaseWidth/2.0);
    }

    private class Wheels implements PIDSource, PIDOutput {

        private SpeedController cim1;
        private SpeedController cim2;
        private Encoder encoder;
        private double prevDist = 0;
        private long prevTime = System.currentTimeMillis();

        public Wheels(Encoder encoder, boolean side, SpeedController cim1, SpeedController cim2) {
            this.encoder = encoder;
            this.cim1 = cim1;
            this.cim2 = cim2;
        }

        public double pidGet() {
            // I would just use the getRate function of the encoder but the wpi
            // code is currently broken
            long curTime = System.currentTimeMillis();
            double curDist = encoder.getDistance();
            double delT = ((double) (curTime - prevTime))/1000.0;
            double v = delT != 0 ? (curDist - prevDist)/delT : 0;
            prevTime = curTime;
            prevDist = curDist;
            return v;
        }

        public void pidWrite(double output) {
            cim1.set(output);
            cim2.set(output);
        }


    }

}
