package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Not Working

public class SanjuPIDController {
    private double kP, kI, kD, kF;
    private double derivative;
    private double target; //in ticks
    private double integralSum;
    private double error; // in ticks
    private double lastError;
    private static final double MAX_INTEGRAL = 1000; // Adjust based on your system's needs

    // declare Mathy Variables
    private int ticksPerRotation = 8192;
    double wheelCircumference = (Math.PI * 60)/25.4;
    double ticksPerInch = (ticksPerRotation / wheelCircumference);

    public SanjuPIDController(double kP, double kI, double kD, double kF)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.integralSum = 0;
        this.lastError = 0;
    }

    public SanjuPIDController(String type)
    {
        if (type.equals("straight"))
        {
            this.kP = 0.00003;
            this.kI = 0;
            this.kD = 0;
            this.kF = 0.15;
        }
        if (type.equals("strafe"))
        {
            this.kP = 0.00008;
            this.kI = 0;
            this.kD = 0.00001;
            this.kF = 0.22;
        }
        if (type.equals("turn"))
        {
            this.kP = 0.003;
            this.kI = 0;
            this.kD = 0;
            this.kF = 0.15;
        }
        this.integralSum = 0;
        this.lastError = 0;
    }

    public void setTarget(double target) //set always in inches
    {
        this.target = target;
    }

    public void setError(double newError)
    {
        error = newError;
    }

    public double calculateOutput(double currentPosition, double deltaTime)
    {
        error = target - currentPosition;

        double aMaxPoint = target/4;

        double proportional = kP * error;

        integralSum += (error * deltaTime);
        // Anti-windup especially when distance is large
        integralSum = Range.clip(integralSum, -MAX_INTEGRAL, MAX_INTEGRAL);
        double integral = kI * integralSum;

        double feedforward = kF * Math.signum(error);

        derivative = kD * (error - lastError)/deltaTime;

        lastError = error;

        double baseOutput = proportional + integral + derivative;
        double output;

        if (target <0)
        {
            if (currentPosition > aMaxPoint)
            {
                output = feedforward + (currentPosition/aMaxPoint) * baseOutput;
            }
            else
            {
                output = feedforward + baseOutput;
            }
        }
        else if (target > 0)
        {
            if (currentPosition < aMaxPoint)
            {
                output = feedforward + (currentPosition/aMaxPoint) * baseOutput;
            }
            else
            {
                output = feedforward + baseOutput;
            }
        }
        else
        {
            output = feedforward + baseOutput;
        }

        output = Range.clip(output, -0.5, 0.5);

        return output;
    }

    public double getDerivative() {
        return derivative;
    }

    public double updateError(double target, double newLoc)
    {
        error = target - newLoc;
        return error;
    }

    public double getError()
    {
        return error;
    }
    public double getTarget()
    {
        return target;
    }
    public double getkF()
    {
        return kF;
    }
    public void reset()
    {
        integralSum = 0;
        lastError = 0;
    }

}