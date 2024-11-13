package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //utilized linear mode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; //send to Autonomous on Driver Hub
import com.qualcomm.robotcore.hardware.DcMotor; //use DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple; //
import com.qualcomm.robotcore.util.ElapsedTime; // use for time

//Problem = position, output, and error all not changing
// PID not working
//Something is zero that shouldn't be zero


@Autonomous(name = "Test", group = "Draft")
public class Test extends LinearOpMode
{
    //declare Motors
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor frontLeft;

    //declare deadWheels
    private DcMotor rightDeadWheel;
    private DcMotor leftDeadWheel;
    private DcMotor centerDeadWheel;

    // reg variables
    private double deltaTime;

    //declare PIDControllers
    private SanjuPIDController centerPIDController, leftPIDController, rightPIDController;

    // declare Mathy Variables
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    // declare motor powers, and wheel distances
    double leftOutput, rightOutput, centerOutput;
    double leftPosition, rightPosition, centerPosition; // in ticks

    @Override
    public void runOpMode() throws InterruptedException
    {
        //method I made below that initialized hardware
        initializeHardware();

        //setting PID Controllers
        leftPIDController = new SanjuPIDController("straight");
        rightPIDController = new SanjuPIDController("straight");
        centerPIDController = new SanjuPIDController("strafe");

        waitForStart(); //FTC SDK method which wait for the "Play"
        telemetry.addData("after wait for start",getRuntime());
        telemetry.update();
        // MAIN PART OF THE CODE!!!!!!!!!!!!
        while (opModeIsActive()) {
            telemetry.addData("Present", "Yes");
            telemetry.update();
            driveStraight(36);
            driveStrafe(36);
            driveStraight(-36);
            driveStrafe(-36);



            break;
        }

    }

    private void driveStraight(double target) //assumes target is zero
    {
        // Reset encoders before starting
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reset and set target for PID controller
        leftPIDController.setTarget(target);
        rightPIDController.setTarget(target);

        runtime.reset();//useless right now, but can use later for telemetry
        timer.reset();
        telemetry.addData("in DriveStraight", "yes");
        telemetry.update();
        while (opModeIsActive() && !isStraightTargetReached()) {
            leftPosition = leftDeadWheel.getCurrentPosition(); //position in ticks
            rightPosition = rightDeadWheel.getCurrentPosition(); //same

            deltaTime = Math.max(timer.milliseconds(), 1e-3); // Avoids setting time to zero, Minimum deltaTime of 1 microsecond
            timer.reset();
            leftOutput = leftPIDController.calculateOutput(leftPosition, deltaTime);
            rightOutput = rightPIDController.calculateOutput(rightPosition, deltaTime);

            setStraightPower(leftOutput, rightOutput);

            // Telemetry for debugging
            telemetry.addData("Left Position", leftPosition);
            telemetry.addData("Right Position", rightPosition);
            telemetry.addData("Left Output", leftOutput);
            telemetry.addData("Right Output", rightOutput);
            telemetry.addData("Runtime", runtime.milliseconds());
            telemetry.addData("Target", leftPIDController.getTarget());
            telemetry.update();
        }
    }

    private void driveStrafe(double target) //strafes to the right
    {
        // Reset encoders before starting
        centerDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reset and set target for PID controller
        centerPIDController.setTarget(target);
        telemetry.addData("Target", centerPIDController.getTarget());
        telemetry.update();

        runtime.reset();//useless right now, but can use later for telemetry
        timer.reset();

        while (opModeIsActive() && !isStrafeTargetReached())
        {
            centerPosition = centerDeadWheel.getCurrentPosition(); //position in ticks

            deltaTime = Math.max(timer.milliseconds(), 1e-3); // Avoids setting time to zero, Minimum deltaTime of 1 microsecond
            timer.reset();
            centerOutput = centerPIDController.calculateOutput(centerPosition, deltaTime);

            setStrafePower(centerOutput);

            // Telemetry for debugging
            telemetry.addData("Center Position", centerPosition); //is negative
            telemetry.addData("Center Output", centerOutput);
            telemetry.addData("Target", centerPIDController.getTarget());
            telemetry.addData("Runtime", runtime.milliseconds());
            telemetry.update();
        }
    }

    private void setStrafePower(double centerMotorPower) // automated to go right
    {
        backLeft.setPower(-centerMotorPower);
        backRight.setPower(centerMotorPower);
        frontRight.setPower(-centerMotorPower);
        frontLeft.setPower(centerMotorPower);
        telemetry.addData("PowerSet","yup");
        telemetry.update();
    }


    private void setStraightPower(double leftMotorPower, double rightMotorPower) //goes the calculated direction
    {
        backLeft.setPower(leftMotorPower);
        backRight.setPower(rightMotorPower);
        frontRight.setPower(rightMotorPower);
        frontLeft.setPower(leftMotorPower);
        telemetry.addData("PowerSet","yup");
        telemetry.update();
    }

    // Check if the target distance is reached
    private boolean isStraightTargetReached()
    {
        telemetry.update();
        double leftError = Math.abs(leftPIDController.getError());
        double rightError = Math.abs(rightPIDController.getError());
        telemetry.addData("leftError",leftError);
        telemetry.addData("rightError", rightError);
        telemetry.update();
        return leftError < 200 && rightError < 200; // Tolerance of 200 encoder counts
    }

    private boolean isStrafeTargetReached()
    {
        double centerError = Math.abs(centerPIDController.getError());
        telemetry.addData("centerError",centerError);
        return centerError < 200; // Tolerance of 200 encoder counts
        //Later make tolerance a variable
    }


    private void initializeHardware()
    {
        // Hardware initialization
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        rightDeadWheel = hardwareMap.get(DcMotor.class, "rightDeadWheel");
        leftDeadWheel = hardwareMap.get(DcMotor.class, "leftDeadWheel");
        centerDeadWheel = hardwareMap.get(DcMotor.class, "centerDeadWheel");

        // Set motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        rightDeadWheel.setDirection(DcMotorSimple.Direction.FORWARD); //Make sure the deadwheels are in the right direction
        leftDeadWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        centerDeadWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //If we ran using encoder, the robot's speed would be automatically adjusted
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //If we don't specify that we are running WITHOUT encoders, it will set the status to what it previously was, leading to unpredictability
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //If we ran using encoder, the robot's speed would be automatically adjusted
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //If we don't specify that we are running WITHOUT encoders, it will set the status to what it previously was, leading to unpredictability
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}