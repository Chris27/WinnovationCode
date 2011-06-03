
package edu.wpi.first.wpilibj.winnovation.utils;


/**
 * PIDController code modified to be threadless i.e. a separate task will not be 
 * generated and it becomes your responsibility to periodically call calculate()
 * and set the output.  I've also modified how the integral term is handled to
 * prevent windup when adjusting the I term and to focus the effect of the I term
 * when you are close to the target and not let the integral of the error explode
 * when you are far away.  This makes it possible to actually slow down and come
 * to a halt at the setpoint instead of just blowing past it at full power.
 *
 * onTarget() has also been modified to consider absolute error remaining rather
 * then a percentage as in the original WPI implementation.
 *
 * @author Chris
 */
public class ThreadlessPID {

    private double m_P;			// factor for "proportional" control
    private double m_I;			// factor for "integral" control
    private double m_D;			// factor for "derivative" control
    private double m_maximumOutput = 1.0;	// |maximum output|
    private double m_minimumOutput = -1.0;	// |minimum output|
    private double m_maximumInput = 0.0;		// maximum input - limit setpoint to this
    private double m_minimumInput = 0.0;		// minimum input - limit setpoint to this
    private boolean m_continuous = false;	// do the endpoints wrap around? eg. Absolute encoder		//is the pid controller enabled
    private double m_prevError = 0.0;	// the prior sensor input (used to compute velocity)
    private double m_totalError = 0.0; //the sum of the errors for use in the integral calc
    private double m_tolerance = 0.05;	//the percetage error that is considered on target
    private double m_setpoint = 0.0;
    private double m_error = 0.0;
    private double m_result = 0.0;

    public ThreadlessPID(double Kp, double Ki, double Kd) {
        m_P = Kp;
        m_I = Ki;
        m_D = Kd;
    }

    public void reset() {
        m_error = 0.0;
        m_result= 0.0;
        m_totalError = 0.0;
        m_prevError = 0.0;
    }

    public double calculate(double setpoint, double input) {
        
        m_error = m_setpoint - input;
        if (m_continuous) {
            if (Math.abs(m_error)
                    > (m_maximumInput - m_minimumInput) / 2) {
                if (m_error > 0) {
                    m_error = m_error - m_maximumInput + m_minimumInput;
                } else {
                    m_error = m_error
                            + m_maximumInput - m_minimumInput;
                }
            }
        }

        return calculate(m_error);
    }

    private int sign(double x) {
        return x >= 0 ? 1 : -1;
    }


    public double calculate(double error) {

        m_error = error;

        if (((m_totalError + m_error) * m_I < m_maximumOutput)
                && ((m_totalError + m_error) * m_I > m_minimumOutput)) {
            m_totalError += m_error;
        }

        // clear the integral if passing the set point
        if(sign(m_prevError) != sign(m_error)) {
            m_totalError = 0;
        }

        m_result = (m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError));
        m_prevError = m_error;

        if (m_result > m_maximumOutput) {
            m_result = m_maximumOutput;
        } else if (m_result < m_minimumOutput) {
            m_result = m_minimumOutput;
        }

        // if going full speed, clear the integral
        if(m_result >= 0.97*m_maximumOutput || m_result <= 0.97*m_minimumOutput) {
            m_totalError = 0;
        }

        return m_result;
    }

    /**
     * Set the PID Controller gain parameters.
     * Set the proportional, integral, and differential coefficients.
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     */
    public  void setPID(double p, double i, double d) {
        m_P = p;
        m_I = i;
        m_D = d;
        m_totalError = 0; // to avoid wind up
    }

    public void setP(double p) {
       m_P = p;
    }
    public void setI(double i) {
        m_I = i;
        m_totalError = 0; // to avoid wind up
    }
    public void setD(double d) {
        m_D = d;
    }

    /**
     * Get the Proportional coefficient
     * @return proportional coefficient
     */
    public double getP() {
        return m_P;
    }

    /**
     * Get the Integral coefficient
     * @return integral coefficient
     */
    public double getI() {
        return m_I;
    }

    /**
     * Get the Differential coefficient
     * @return differential coefficient
     */
    public  double getD() {
        return m_D;
    }

    /**
     * Return the current PID result
     * This is always centered on zero and constrained the the max and min outs
     * @return the latest calculated output
     */
    public  double get() {
        return m_result;
    }

    /**
     *  Set the PID controller to consider the input to be continuous,
     *  Rather then using the max and min in as constraints, it considers them to
     *  be the same point and automatically calculates the shortest route to
     *  the setpoint.
     * @param continuous Set to true turns on continuous, false turns off continuous
     */
    public  void setContinuous(boolean continuous) {
        m_continuous = continuous;
    }

    /**
     *  Set the PID controller to consider the input to be continuous,
     *  Rather then using the max and min in as constraints, it considers them to
     *  be the same point and automatically calculates the shortest route to
     *  the setpoint.
     */
    public  void setContinuous() {
        this.setContinuous(true);
    }

    /**
     * Sets the maximum and minimum values expected from the input.
     *
     * @param minimumInput the minimum value expected from the input
     * @param maximumInput the maximum value expected from the output
     */
    public  void setInputRange(double minimumInput, double maximumInput) {
        if (minimumInput > maximumInput) {
            throw new IllegalArgumentException("Lower bound is greater than upper bound");
        }
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
        setSetpoint(m_setpoint);
    }

    /**
     * Sets the minimum and maximum values to write.
     *
     * @param minimumOutput the minimum value to write to the output
     * @param maximumOutput the maximum value to write to the output
     */
    public  void setOutputRange(double minimumOutput, double maximumOutput) {
        if (minimumOutput > maximumOutput) {
            throw new IllegalArgumentException("Lower bound is greater than upper bound");
        }
        m_minimumOutput = minimumOutput;
        m_maximumOutput = maximumOutput;
    }

    /**
     * Set the setpoint for the PIDController
     * @param setpoint the desired setpoint
     */
    public  void setSetpoint(double setpoint) {
        if (m_maximumInput > m_minimumInput) {
            if (setpoint > m_maximumInput) {
                m_setpoint = m_maximumInput;
            } else if (setpoint < m_minimumInput) {
                m_setpoint = m_minimumInput;
            } else {
                m_setpoint = setpoint;
            }
        } else {
            m_setpoint = setpoint;
        }
    }

    /**
     * Returns the current setpoint of the PIDController
     * @return the current setpoint
     */
    public  double getSetpoint() {
        return m_setpoint;
    }

    /**
     * Returns the current difference of the input from the setpoint
     * @return the current error
     */
    public  double getError() {
        return m_error;
    }

    /**
     * Set the error which is considered tolerable for use with
     * OnTarget.
     * @param error amount which is tolerable
     */
    public  void setTolerance(double error) {
        m_tolerance = error;
    }

    /**
     * Return true if the error is within the amount allowable
     * determined by setTolerance.
     * @return true if the error is less than the tolerance
     */
    public  boolean onTarget() {
        return (Math.abs(m_error) < m_tolerance); 
    }

}
