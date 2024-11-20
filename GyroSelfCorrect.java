package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //utilized linear mode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; //send to Autonomous on Driver Hub
import com.qualcomm.robotcore.hardware.DcMotor; //use DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple; //
import com.qualcomm.robotcore.util.ElapsedTime; // use for time
import com.qualcomm.hardware.bosch.BNO055IMU;


//Problem = position, output, and error all not changing
// PID not working
//Something is zero that shouldn't be zero




@Autonomous(name = "GyroSelfCorrect", group = "Draft")
public class GyroSelfCorrect extends LinearOpMode
{
    //declare Motors
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor frontLeft;


    //declare gadgets
    private DcMotor rightDeadWheel;
    private DcMotor leftDeadWheel;
    private BNO055IMU imu;
    double heading;
    double turnSpeed = 0.4;


    @Override
    public void runOpMode() throws InterruptedException
    {
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


        waitForStart();


        while (opModeIsActive())
        {
            heading = imu.getAngularOrientation().firstAngle;
            telemetry.addData("heading", heading);
            telemetry.update();


            if (Math.abs(heading) > 2) {
                correctHeading(heading);
            }


            sleep(2000);
        }


    }


    private void correctHeading(double heading) {
        while (opModeIsActive() && Math.abs(heading) > 2) {
            // Determine turn direction and set motor power
            if (heading < 0) {
                // Turn left
                setMotorPowers(-turnSpeed, turnSpeed);
            } else {
                // Turn right
                setMotorPowers(turnSpeed, -turnSpeed);
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


    private void setMotorPowers(double leftPower, double rightPower) {
        backLeft.setPower(leftPower);
        frontLeft.setPower(leftPower);
        backRight.setPower(rightPower);
        frontRight.setPower(rightPower);
    }


    private void stopMotors() {
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
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


        // Set motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);


        rightDeadWheel.setDirection(DcMotorSimple.Direction.FORWARD); //Make sure the deadwheels are in the right direction
        leftDeadWheel.setDirection(DcMotorSimple.Direction.REVERSE);


        // Reset encoders
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //If we ran using encoder, the robot's speed would be automatically adjusted
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //If we don't specify that we are running WITHOUT encoders, it will set the status to what it previously was, leading to unpredictability
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rightDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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