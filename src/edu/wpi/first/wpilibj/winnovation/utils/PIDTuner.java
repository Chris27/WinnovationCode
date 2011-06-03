/*
 * PID Tuner
 *
 * This allows PID gains to be tuned using a joystick
 */
package edu.wpi.first.wpilibj.winnovation.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SmartDashboard;

/**
 *
 * @author Tom
 * modified by Chris
 */
public class PIDTuner {

    private Joystick js = null;
    private double kp, ki, kd;
    public int cp, ci, cd;
    private PulseTriggerBoolean pTrigUp = new PulseTriggerBoolean();
    private PulseTriggerBoolean iTrigUp = new PulseTriggerBoolean();
    private PulseTriggerBoolean dTrigUp = new PulseTriggerBoolean();
    private PulseTriggerBoolean pTrigD = new PulseTriggerBoolean();
    private PulseTriggerBoolean iTrigD = new PulseTriggerBoolean();
    private PulseTriggerBoolean dTrigD = new PulseTriggerBoolean();
    private PulseTriggerBoolean reset = new PulseTriggerBoolean();
    private PIDTunable tunable;

    private int numResets = 0;

    public PIDTuner(int port, PIDTunable tunable, double kp, double ki, double kd, int cp, int ci, int cd) {
        this.js = new Joystick(port);
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.cp = cp;
        this.ci = ci;
        this.cd = cd;
        this.tunable = tunable;

        tunable.setKp(cp * kp);
        tunable.setKi(ci * ki);
        tunable.setKd(cd * kd);
    }

    public PIDTuner(int port, PIDTunable tunable, int cp, int ci, int cd) {
        this.js = new Joystick(port);
        this.kp = tunable.getKp();
        this.ki = tunable.getKi();
        this.kd = tunable.getKd();
        this.cp = cp;
        this.ci = ci;
        this.cd = cd;
        this.tunable = tunable;

        tunable.setKp(cp * kp);
        tunable.setKi(ci * ki);
        tunable.setKd(cd * kd);
    }

    public PIDTuner(int port, PIDTunable tunable) {
        this(port, tunable, 0, 0, 0);
    }

    public boolean reset() {
        reset.set(js.getRawButton(10));
        boolean result = reset.get();
        if(result) {
            numResets++;
            SmartDashboard.log(numResets, "times reset");
        }
        return result;
    }

    public void handle() {

        pTrigUp.set(js.getRawButton(7));
        pTrigD.set(js.getRawButton(5));

        iTrigUp.set(js.getRawButton(8));
        iTrigD.set(js.getRawButton(6));

        dTrigUp.set(js.getRawButton(1));
        dTrigD.set(js.getRawButton(3));

        if (pTrigUp.get()) {
            cp++;
            tunable.setKp(cp * kp);
        } else if (pTrigD.get()) {
            cp--;
            tunable.setKp(cp * kp);
        }

        if (iTrigUp.get()) {
            ci++;
            tunable.setKi(ci * ki);
        } else if (iTrigD.get()) {
            ci--;
            tunable.setKi(ci * ki);
        }

        if (dTrigUp.get()) {
            cd++;
            tunable.setKd(cd * kd);
        } else if (dTrigD.get()) {
            cd--;
            tunable.setKd(cd * kd);
        }

        SmartDashboard.log(cp, "PIDTuner cP");
        SmartDashboard.log(ci, "PIDTuner cI");
        SmartDashboard.log(cd, "PIDTuner cD");

    }

    public double getKP() {
        return kp;
    }

    public double getKI() {
        return ki;
    }

    public double getKD() {
        return kd;
    }
}
