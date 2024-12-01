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
    private SanjuPIDController centerPIDController, leftPIDController, rightPIDController, turnPIDController;

    // declare Mathy Variables
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    // declare motor powers, and wheel distances
    double leftOutput, rightOutput, centerOutput;
    double leftPosition, rightPosition, centerPosition; // in ticks

    //gyro stuff
    private BNO055IMU imu;
    double heading;
    double turnSpeed = 0.14;

    //mathy variables
    // declare Mathy Variables
    private int ticksPerRotation = 8192;
    double wheelCircumference = (Math.PI * 60)/25.4;
    double ticksPerInch = (ticksPerRotation / wheelCircumference);

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
        turnPIDController = new SanjuPIDController("turn");

        waitForStart(); //FTC SDK method which wait for the "Play"
        // MAIN PART OF THE CODE!!!!!!!!!!!!
        while (opModeIsActive()) {
            //driveToPosition(0,36);
            //driveToPosition(36,0);
            //driveToPosition(0,-36);
            driveToPosition(-36,0);
            //driveToPosition(12,12);
            //driveTurn(90);
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
            heading = imu.getAngularOrientation().firstAngle;
        }
        stopMotors();
    }

    private void driveStraight(double targetY, double leftErrorY, double rightErrorY) //input in ticks
    {
        leftPIDController.setTarget(targetY); // Forward motion is Y
        rightPIDController.setTarget(targetY);
        leftPIDController.setError(leftErrorY); // Forward motion is Y
        rightPIDController.setError(rightErrorY);
        timer.reset();

        while (opModeIsActive() && !isStraightTargetReached()) {
            leftPosition = leftDeadWheel.getCurrentPosition(); //position in ticks
            rightPosition = rightDeadWheel.getCurrentPosition(); //same
            deltaTime = Math.max(timer.milliseconds(), 1e-3); // Avoids setting time to zero, Minimum deltaTime of 1 microsecond
            timer.reset();
            leftOutput = leftPIDController.calculateOutput(leftPosition, deltaTime);
            rightOutput = rightPIDController.calculateOutput(rightPosition, deltaTime);
            setStraightPower(leftOutput, rightOutput);
        }
        stopMotors();
    }

    private void driveStrafe(double targetX, double errorX) //input in ticks, + strafes to the right
    {
        centerPIDController.setTarget(targetX); // Strafing motion is X
        centerPIDController.setError(errorX); // Strafing motion is X
        timer.reset();

        while (opModeIsActive() && !isStrafeTargetReached())
        {
            centerPosition = centerDeadWheel.getCurrentPosition(); //position in ticks

            deltaTime = Math.max(timer.milliseconds(), 1e-3); // Avoids setting time to zero, Minimum deltaTime of 1 microsecond
            timer.reset();
            centerOutput = centerPIDController.calculateOutput(centerPosition, deltaTime);

            setStrafePower(centerOutput);
        }
        stopMotors();
    }


    private void driveToPosition(double initialTargetX, double initialTargetY) {//input in inches
        // Reset encoders before starting
        double targetX = initialTargetX * ticksPerInch;
        double targetY = initialTargetY * ticksPerInch;

        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (targetX == 0)
        {
            driveStraight(targetY, targetY, targetY);
            centerPosition = centerDeadWheel.getCurrentPosition(); //position in ticks
            double centerUpdatedError = centerPIDController.updateError(targetX, centerPosition);
            telemetry.addData("center updated error", centerUpdatedError);
            telemetry.update();
            if (Math.abs(centerUpdatedError) > 1100) {
                sleep(2000);
                driveStrafe(targetX, centerUpdatedError);
            }
            checkHeading();
        }
        else {
            driveStrafe(targetX, targetX);
            centerPosition = centerDeadWheel.getCurrentPosition(); //position in ticks
            double centerUpdatedError = centerPIDController.updateError(targetX, centerPosition);
            telemetry.addData("center updated error", centerUpdatedError);
            leftPosition = leftDeadWheel.getCurrentPosition(); //position in ticks
            rightPosition = rightDeadWheel.getCurrentPosition(); //same
            double leftUpdatedError = leftPIDController.updateError(targetY, leftPosition);
            double rightUpdatedError = rightPIDController.updateError(targetY, rightPosition);
            telemetry.addData("left updated error", leftUpdatedError);
            telemetry.addData("right updated error", rightUpdatedError);
            telemetry.update();
            if (Math.abs(leftUpdatedError) > 1100 || Math.abs(rightUpdatedError) > 1100) {
                sleep(2000);
                driveStraight(targetY, leftUpdatedError, rightUpdatedError);
            }
            centerPosition = centerDeadWheel.getCurrentPosition(); //position in ticks
            centerUpdatedError = centerPIDController.updateError(targetX, centerPosition);
            telemetry.addData("center updated error", centerUpdatedError);
            leftPosition = leftDeadWheel.getCurrentPosition(); //position in ticks
            rightPosition = rightDeadWheel.getCurrentPosition(); //same
            leftUpdatedError = leftPIDController.updateError(targetY, leftPosition);
            rightUpdatedError = rightPIDController.updateError(targetY, rightPosition);
            telemetry.addData("AFTER UPDATE left", leftUpdatedError);
            telemetry.addData("AFTER UPDATE right", rightUpdatedError);
            telemetry.update();
            sleep(2000);
            checkHeading();
            centerPosition = centerDeadWheel.getCurrentPosition(); //position in ticks
            centerUpdatedError = centerPIDController.updateError(targetX, centerPosition);
            telemetry.addData("center updated error", centerUpdatedError);
            leftPosition = leftDeadWheel.getCurrentPosition(); //position in ticks
            rightPosition = rightDeadWheel.getCurrentPosition(); //same
            leftUpdatedError = leftPIDController.updateError(targetY, leftPosition);
            rightUpdatedError = rightPIDController.updateError(targetY, rightPosition);
            telemetry.addData("AFTER CHECK left", leftUpdatedError);
            telemetry.addData("AFTER CHECK right", rightUpdatedError);
            telemetry.update();
            sleep(2000);
        }

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

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

        }
        stopMotors();
        checkHeading();
    }

    private void driveTurn(double targetAngle)
    {
        turnPIDController.setTarget(targetAngle); // Strafing motion is X
        turnPIDController.setError(targetAngle); // Strafing motion is X
        timer.reset();

        while (opModeIsActive() && !isTurnTargetReached()) {
            heading = -(imu.getAngularOrientation().firstAngle);

            deltaTime = Math.max(timer.milliseconds(), 1e-3); // Avoids setting time to zero, Minimum deltaTime of 1 microsecond
            timer.reset();

            double power = turnPIDController.calculateOutput(heading, deltaTime);
            // Determine turn direction and set motor power
            setTurnPower(power);
            heading = imu.getAngularOrientation().firstAngle;
            telemetry.addData("target", targetAngle);
            telemetry.addData("heading", heading);
            telemetry.addData("power",power);
            telemetry.update();
        }
        stopMotors();
        telemetry.addData("heading",heading);
        telemetry.update();
        sleep(2000);
    }

    private void setTurnPower(double turnMotorPower)
    {
        frontLeft.setPower(turnMotorPower);
        backLeft.setPower(turnMotorPower);
        frontRight.setPower(-turnMotorPower);
        backRight.setPower(-turnMotorPower);
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
    }


    private void setStraightPower(double leftMotorPower, double rightMotorPower) //goes the calculated direction
    {
        backLeft.setPower(leftMotorPower);
        backRight.setPower(rightMotorPower);
        frontRight.setPower(rightMotorPower);
        frontLeft.setPower(leftMotorPower);
    }

    // Check if the target distance is reached
    private boolean isStraightTargetReached()
    {
        double leftError = Math.abs(leftPIDController.getError());
        double rightError = Math.abs(rightPIDController.getError());

        return leftError < 50 && rightError < 50; // Tolerance of 50 encoder counts
    }

    private boolean isStrafeTargetReached()
    {
        double centerError = Math.abs(centerPIDController.getError());

        return centerError < 50; // Tolerance of 50 encoder counts
        //Later make tolerance a variable
    }

    private boolean isTurnTargetReached()
    {
        double turnError = Math.abs(turnPIDController.getError());

        return turnError < 0.3; // 1 degree
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