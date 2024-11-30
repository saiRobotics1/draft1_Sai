package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //utilized linear mode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; //send to Autonomous on Driver Hub
import com.qualcomm.robotcore.hardware.DcMotor; //use DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple; //
import com.qualcomm.robotcore.util.ElapsedTime; // use for time
import com.qualcomm.robotcore.util.Range;

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

    //gyro stuff
    private BNO055IMU imu;
    double heading;
    double turnSpeed = 0.12;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //method I made below that initialized hardware
        initializeHardware();

        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();

        // Wait for calibration to complete or for stop request
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(100); // Short delay to avoid overloading the CPU
            idle();    // Let other processes run
        }

        telemetry.addData("Status", "IMU Calibrated");
        telemetry.update();

        //setting PID Controllers
        leftPIDController = new SanjuPIDController("straight");
        rightPIDController = new SanjuPIDController("straight");
        centerPIDController = new SanjuPIDController("strafe");

        waitForStart(); //FTC SDK method which wait for the "Play"
        telemetry.addData("after wait for start",getRuntime());
        telemetry.update();
        // MAIN PART OF THE CODE!!!!!!!!!!!!
        while (opModeIsActive()) {
            driveToPosition(0,36);
            driveToPosition(36,0);
            driveToPosition(0,-36);
            driveToPosition(-36,0);
            break;
        }

    }

    private void checkHeading()
    {
        heading = imu.getAngularOrientation().firstAngle;
        if (Math.abs(heading) > 1) {
            correctHeading(heading);
        }
    }
    private void correctHeading(double heading) {
        while (opModeIsActive() && Math.abs(heading) > 1) {
            // Determine turn direction and set motor power
            if (heading < 0) {
                // Turn left
                setStraightPower(-turnSpeed, turnSpeed);
            } else {
                // Turn right
                setStraightPower(turnSpeed, -turnSpeed);
            }


            // Update heading
            heading = imu.getAngularOrientation().firstAngle;
            telemetry.addData("Correcting Heading", heading);
            telemetry.update();
        }


        // Stop motors after correction
        stopMotors();
        telemetry.addData("Correction Complete", "Heading: %.2f", heading);
        telemetry.update();
    }
    private void driveStraight(double target) //assumes target is zero
    {
        // Reset encoders before starting
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reset and set target for PID controller
        leftPIDController.setTarget(target);
        rightPIDController.setTarget(target);
        leftPIDController.setError(target);
        rightPIDController.setError(target);
        telemetry.addData("Target",leftPIDController.getTarget());

        runtime.reset();//useless right now, but can use later for telemetry
        timer.reset();
        telemetry.update();

        while (opModeIsActive() && !isStraightTargetReached()) {
            leftPosition = leftDeadWheel.getCurrentPosition(); //position in ticks
            rightPosition = rightDeadWheel.getCurrentPosition(); //same
            telemetry.addData("Position",leftPosition);
            telemetry.update();
            deltaTime = Math.max(timer.milliseconds(), 1e-3); // Avoids setting time to zero, Minimum deltaTime of 1 microsecond
            timer.reset();
            leftOutput = leftPIDController.calculateOutput(leftPosition, deltaTime);
            rightOutput = rightPIDController.calculateOutput(rightPosition, deltaTime);
            setStraightPower(leftOutput, rightOutput);
        }
        stopMotors();
        checkHeading();
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
            telemetry.addData("Center Derivative", centerPIDController.getDerivative());

            telemetry.addData("Center Position", centerPosition); //is negative
            telemetry.addData("Target", centerPIDController.getTarget());
            telemetry.update();
        }
        stopMotors();
        checkHeading();
    }

    private void driveDiagonal(double targetX, double targetY) {
        // Reset encoders before starting
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reset and set target for PID controllers
        leftPIDController.setTarget(targetY); // Forward motion is Y
        rightPIDController.setTarget(targetY);
        centerPIDController.setTarget(targetX); // Strafing motion is X

        timer.reset();

        while (opModeIsActive() && (!isStraightTargetReached() || !isStrafeTargetReached())) {
            // Get current positions
            leftPosition = leftDeadWheel.getCurrentPosition();
            rightPosition = rightDeadWheel.getCurrentPosition();
            centerPosition = centerDeadWheel.getCurrentPosition();

            // Calculate delta time
            deltaTime = Math.max(timer.milliseconds(), 1e-3); // Avoid divide-by-zero errors
            timer.reset();

            // PID outputs
            double forwardLeftOutput = leftPIDController.calculateOutput(leftPosition, deltaTime);
            double forwardRightOutput = rightPIDController.calculateOutput(rightPosition, deltaTime);
            double strafeOutput = centerPIDController.calculateOutput(centerPosition, deltaTime);

            // Combine motor powers
            double backLeftPower = forwardLeftOutput - strafeOutput;
            backLeftPower = Range.clip(backLeftPower, -0.8, 0.8);
            double backRightPower = forwardRightOutput + strafeOutput;
            backRightPower = Range.clip(backRightPower, -0.8, 0.8);
            double frontRightPower = forwardRightOutput - strafeOutput;
            frontRightPower = Range.clip(frontRightPower, -0.8, 0.8);
            double frontLeftPower = forwardLeftOutput + strafeOutput;
            frontLeftPower = Range.clip(frontLeftPower, -0.8, 0.8);

            telemetry.update();
            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // Debugging telemetry
            telemetry.addData("Forward Left Output", forwardLeftOutput);
            telemetry.addData("Strafe Output", strafeOutput);
            telemetry.addData("Left Error", leftPIDController.getError());
            telemetry.addData("Right Error", rightPIDController.getError());
            telemetry.addData("Center Error", centerPIDController.getError());
            telemetry.update();
        }
        stopMotors();
        checkHeading();
    }

    private void driveToPosition(double targetX, double targetY) {
        // Reset encoders before starting
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveStrafe(targetY);
        checkHeading();
        leftPosition = leftDeadWheel.getCurrentPosition(); //position in ticks
        rightPosition = rightDeadWheel.getCurrentPosition(); //same
        leftPIDController.updateError(leftPosition);
        rightPIDController.updateError(rightPosition);
        driveStraight(targetX);
        checkHeading();
    }

    private void stopMotors()
    {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
    }

    private void setStrafePower(double centerMotorPower) // automated to go right
    {
        double sign = Math.signum(centerMotorPower);
        backLeft.setPower(-centerMotorPower);
        backRight.setPower(centerMotorPower);
        frontRight.setPower(-centerMotorPower);
        frontLeft.setPower(centerMotorPower);
        telemetry.update();
    }


    private void setStraightPower(double leftMotorPower, double rightMotorPower) //goes the calculated direction
    {
        backLeft.setPower(leftMotorPower);
        backRight.setPower(rightMotorPower);
        frontRight.setPower(rightMotorPower);
        frontLeft.setPower(leftMotorPower);
        telemetry.update();
    }

    // Check if the target distance is reached
    private boolean isStraightTargetReached()
    {
        double leftError = Math.abs(leftPIDController.getError());
        double rightError = Math.abs(rightPIDController.getError());

        telemetry.addData("leftError",leftPIDController.getError());
        telemetry.addData("rightError", rightPIDController.getError());
        telemetry.update();
        return leftError < 50 && rightError < 50; // Tolerance of 50 encoder counts
    }

    private boolean isStrafeTargetReached()
    {
        double centerError = Math.abs(centerPIDController.getError());
        telemetry.addData("centerError",centerError);
        telemetry.update();

        return centerError < 50; // Tolerance of 50 encoder counts
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

        // Initialize the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }
}