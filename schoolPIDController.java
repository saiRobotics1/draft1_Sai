package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Not Working

public class SanjuPIDController {
    private double kP, kI, kD, kF;
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
            this.kP = 0.0005;
            this.kI = 0;
            this.kD = 0.;
            this.kF = 0.1;
        }
        if (type.equals("strafe"))
        {
            this.kP = 0;
            this.kI = 0;
            this.kD = 0;
            this.kF = 0;
        }
        this.integralSum = 0;
        this.lastError = 0;
    }

    public void setTarget(double target) //set always in inches
    {
        this.target = target * ticksPerInch;
        error = target;
    }

    public double calculateOutput(double currentPosition, double deltaTime)
    {
        error = target - currentPosition;

        //useful if one motor reaches destination but other doesn't
        /*if (Math.abs(error) < 10) {  //10 is the tolerance
            return 0; // Stop power once target is reached
        }*/

        double aMaxPoint = target/4;

        double proportional = kP * error;

        integralSum += (error * deltaTime);
        // Anti-windup especially when distance is large
        integralSum = Range.clip(integralSum, -MAX_INTEGRAL, MAX_INTEGRAL);
        double integral = kI * integralSum;

        double derivative = kD * (error - lastError)/deltaTime;
        lastError = error;

        double baseOutput = proportional + integral + derivative;
        double output;

        if (currentPosition < aMaxPoint)
        {
            output = kF + (currentPosition/aMaxPoint) * baseOutput;
        }
        else
        {
            output = kF + baseOutput;
        }

        output = Range.clip(output, -0.8, 0.8);

        return output;
    }

    public double getError()
    {
        return error;
    }

    public void reset()
    {
        integralSum = 0;
        lastError = 0;
    }

}