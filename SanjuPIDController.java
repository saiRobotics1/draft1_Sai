package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class SanjuPIDController {
    private double kP, kI, kD, kF;
    private double target; //in ticks
    private double integralSum;
    private double error; // in ticks
    private double lastError;
    private static final double MAX_INTEGRAL = 1000; // Adjust based on your system's needs

    // declare Mathy Variables
    private ElapsedTime timer;
    private int ticksPerRotation = 8192;
    double wheelCircumference = (Math.PI * 60);
    double inchesPerTick = (wheelCircumference / ticksPerRotation) / 2.54;
    double ticksPerInch = (ticksPerRotation / wheelCircumference) * 25.4;

    public SanjuPIDController(double kP, double kI, double kD, double kF)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.integralSum = 0;
        this.lastError = 0;
        this.timer = new ElapsedTime();
    }

    public SanjuPIDController(String type)
    {
        if (type.equals("straight"))
        {
            this.kP = 0;
            this.kI = 0;
            this.kD = 0;
            this.kF = 0;
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
        this.timer = new ElapsedTime();
    }

    public void setTarget(double target) //set always in inches
    {
        this.target = target * ticksPerInch;
    }

    public double calculateOutput(double currentPosition, double deltaTime)
    {
        error = target - currentPosition;

        //useful if one motor reaches destination but other doesn't
        if (Math.abs(error) < 200) {  //200 is the tolerance
            return 0; // Stop power once target is reached
        }

        double aMaxPoint = target/4;

        double proportional = kP * error;
        
        integralSum += (error * deltaTime);
        // Anti-windup especially when distance is large
        integralSum = Range.clip(integralSum, -MAX_INTEGRAL, MAX_INTEGRAL);
        double integral = kI * integralSum;
        
        double derivative = kD * (error - lastError)/deltaTime;
        lastError = error;

        double baseOutput = proportional + integral + derivative;;
        double output;

        if (currentPosition < aMaxPoint)
        {
            output = kF + (currentPosition/aMaxPoint) * baseOutput;
        }
        else
        {
            output = kF + baseOutput;
        }

        output = Range.clip(output, -0.5, 0.5);
        
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
        timer.reset();
    }

}
