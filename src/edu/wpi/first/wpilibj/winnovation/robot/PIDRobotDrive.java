package edu.wpi.first.wpilibj.winnovation.robot;

import edu.wpi.first.wpilibj.Encoder;
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
    
    private PIDController lController;
    private PIDController rController;

    // PID constants
    private double Kp = 1.0;
    private double Ki = 0.0;
    private double Kd = 0.0;


    public PIDRobotDrive(Encoder lEncoder, Encoder rEncoder,
            SpeedController lCim1, SpeedController lCim2, SpeedController rCim1,
            SpeedController rCim2) {

        super(lCim1, lCim2, rCim1, rCim2);

        lEncoder.setDistancePerPulse(2.0*Math.PI*Constants.WheelRadius/Constants.EncoderTicksPerRev);
        Wheels leftWheels = new Wheels(lEncoder, lCim1, lCim2);
        Wheels rightWheels = new Wheels(rEncoder, rCim1, rCim2);
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

    public void tankDrive(double leftVel, double rightVel) {
        lController.setSetpoint(leftVel);
        rController.setSetpoint(rightVel);
    }

    public void drive(double vel, double curve) {
        lController.setSetpoint(vel - curve*vel*Constants.WheelBaseRadius);
        rController.setSetpoint(vel + curve*vel*Constants.WheelBaseRadius);
    }

    private class Wheels implements PIDSource, PIDOutput {

        private SpeedController cim1;
        private SpeedController cim2;
        private Encoder encoder;

        public Wheels(Encoder encoder, SpeedController cim1, SpeedController cim2) {
            this.encoder = encoder;
            this.cim1 = cim1;
            this.cim2 = cim2;
        }

        public double pidGet() {
            return encoder.getRate();
        }

        public void pidWrite(double output) {
            cim1.set(output);
            cim2.set(output);
        }


    }

}
