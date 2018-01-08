package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class EdgeBot {
    // Declare motors
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor rearLeftMotor = null;
    public DcMotor rearRightMotor = null;

    public DcMotor liftMotor = null;

    public DcMotor craneExtensionMotor = null;

    public DcMotor motor7 = null;
    public DcMotor motor8 = null;

    // Declare servos
    public Servo jewelLiftServo = null;
    public Servo jewelFlipServo = null;

    public Servo clampServoLeft = null;
    public Servo clampServoRight = null;

    public CRServo craneRotateServo = null;
    public Servo clawWristServo = null;
    public Servo clawPinchServo = null;

    public Servo servo8 = null;
    public Servo servo9 = null;
    public Servo servo10 = null;
    public Servo servo11 = null;
    public Servo servo12 = null;

    // Declare imu (inertial motion unit)
    public BNO055IMU imu = null;

    // Declare distance sensor
    public ModernRoboticsI2cRangeSensor distanceSensor = null;

    // Declare color sensors
    public ColorSensor leftColorSensor = null;
    public ColorSensor rightColorSensor = null;

    // Declare color sensor LEDs
    public DigitalChannel leftLED = null;
    public DigitalChannel rightLED = null;

    // Local OpMode members
    private HardwareMap hMap;
    private ElapsedTime localPeriod = new ElapsedTime();

    // A reference to the current opMode
    public LinearOpMode currentOpmode;

    // Constructor
    public EdgeBot() {

    }

    // Initializes the hardware interfaces -- ONLY call this in runOpMode()
    public void init(HardwareMap map, LinearOpMode opMode) {
        this.hMap = map;

        currentOpmode = opMode;

        // Initialize the drive motors
        frontLeftMotor = hMap.dcMotor.get("frontleft");
        frontRightMotor = hMap.dcMotor.get("frontright");
        rearLeftMotor = hMap.dcMotor.get("rearleft");
        rearRightMotor = hMap.dcMotor.get("rearright");

        setDriveMotorsRunUsingEncoders();

        // Initialize the lift motor
        liftMotor = hMap.dcMotor.get("liftmotor");

        // Initialize the crane extension motor
        craneExtensionMotor = hMap.dcMotor.get("cranemotor");

        // Initialize the jewel arm servos
        jewelLiftServo = hMap.servo.get("jewellift");
        jewelFlipServo = hMap.servo.get("jewelflip");

        resetJewelServos();

        // Initialize the clamp servos
        clampServoLeft = hMap.servo.get("leftclamp");
        clampServoRight = hMap.servo.get("rightclamp");

        openClampServos();

        // Initialize the crane servos
        craneRotateServo = hMap.crservo.get("cranerotate");
        clawWristServo = hMap.servo.get("clawwrist");
        clawPinchServo = hMap.servo.get("clawpinch");

        clawWristDown();
        clawPinch();

        // Initialize the imu
        imu = hMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu.initialize(parameters);

        // Initialize the distance sensor
        distanceSensor = hMap.get(ModernRoboticsI2cRangeSensor.class, "distance");

        // Initialize the color sensors
        leftColorSensor = hMap.colorSensor.get("colorleft");
        rightColorSensor = hMap.colorSensor.get("colorright");

        // Initialize the LED outputs
        /*
        leftLED = hMap.digitalChannel.get("lLED");
        rightLED = hMap.digitalChannel.get("rLED");

        leftLED.setMode(DigitalChannel.Mode.OUTPUT);
        rightLED.setMode(DigitalChannel.Mode.OUTPUT);

        turnOffLEDs();
        */
    }

    // Waits until a certain time has elapsed since the last call
    public void waitForTick(long periodMilliseconds) {
        long remaining = periodMilliseconds - (long) localPeriod.milliseconds();

        // Sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        localPeriod.reset();
    }

    // Stop all of the drive motors
    public void stopDriveMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }

    // Converts joystick inputs to motor powers for mecanum wheels
    public void mecanumDrive(double leftX, double leftY, double rightX, Telemetry telemetry) {
        // Set motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveMotorsRunUsingEncoders();

        // The vector sum of the x and y positions of the left joystick
        double velocity = Math.min(Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), 1);
        double heading = Math.atan2(-leftX, leftY);
        double turnVelocity = -rightX;

        // Convert velocity and heading to motor powers
        double frontLeft = velocity * Math.sin(-heading + Math.PI / 4) + turnVelocity;
        double frontRight  = velocity * Math.cos(-heading + Math.PI / 4) - turnVelocity;
        double rearLeft = velocity * Math.cos(-heading + Math.PI / 4) + turnVelocity;
        double rearRight = velocity * Math.sin(-heading + Math.PI / 4) - turnVelocity;

        // Scale the motor powers with 1 as the max
        double max1 = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
        double max2 = Math.max(Math.abs(rearLeft), Math.abs(rearRight));
        double maxValue = Math.max(max1, max2);

        if (maxValue > 1.0) {
            frontLeft /= maxValue;
            frontRight /= maxValue;
            rearLeft /= maxValue;
            rearRight /= maxValue;
        }

        // Set the motors to the calculated powers
        frontLeftMotor.setPower(frontLeft);
        frontRightMotor.setPower(frontRight);
        rearLeftMotor.setPower(rearLeft);
        rearRightMotor.setPower(rearRight);

        telemetry.addLine("Motor powers")
                .addData("Front left", frontLeft)
                .addData("Front right", frontRight)
                .addData("Rear left", rearLeft)
                .addData("Rear right", rearRight);

        telemetry.addData("Heading", heading);
    }

    public void driveBackwardForSteps(int numberOfSteps, double speed) {
        // Set motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Calculate step counts
        int frontLeftMotorStepsToDo = frontLeftMotor.getCurrentPosition() + numberOfSteps;
        int frontRightMotorStepsToDo = frontRightMotor.getCurrentPosition() + numberOfSteps;
        int rearLeftMotorStepsToDo = rearLeftMotor.getCurrentPosition() + numberOfSteps;
        int rearRightMotorStepsToDo = rearRightMotor.getCurrentPosition() + numberOfSteps;

        // Set target steps
        frontLeftMotor.setTargetPosition(frontLeftMotorStepsToDo);
        frontRightMotor.setTargetPosition(frontRightMotorStepsToDo);
        rearLeftMotor.setTargetPosition(rearLeftMotorStepsToDo);
        rearRightMotor.setTargetPosition(rearRightMotorStepsToDo);

        // Turn on RUN_TO_POSITION
        setDriveMotorsRunToPosition();

        // Start motion
        setDriveMotorsToCommonSpeed(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (frontRightMotor.isBusy() && frontLeftMotor.isBusy() && rearRightMotor.isBusy() && rearLeftMotor.isBusy() && currentOpmode.opModeIsActive()) {
            waitForTick(50);
        }

        // Stop all motion;
        stopDriveMotors();

        // Turn off RUN_TO_POSITION
        setDriveMotorsRunUsingEncoders();
    }

    public void driveForwardForSteps(int numberOfSteps, double speed) {
        // Set motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Calculate step counts
        int frontLeftMotorStepsToDo = frontLeftMotor.getCurrentPosition() + numberOfSteps;
        int frontRightMotorStepsToDo = frontRightMotor.getCurrentPosition() + numberOfSteps;
        int rearLeftMotorStepsToDo = rearLeftMotor.getCurrentPosition() + numberOfSteps;
        int rearRightMotorStepsToDo = rearRightMotor.getCurrentPosition() + numberOfSteps;

        // Set target steps
        frontLeftMotor.setTargetPosition(frontLeftMotorStepsToDo);
        frontRightMotor.setTargetPosition(frontRightMotorStepsToDo);
        rearLeftMotor.setTargetPosition(rearLeftMotorStepsToDo);
        rearRightMotor.setTargetPosition(rearRightMotorStepsToDo);

        // Turn on RUN_TO_POSITION
        setDriveMotorsRunToPosition();

        // Start motion
        setDriveMotorsToCommonSpeed(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (frontRightMotor.isBusy() && frontLeftMotor.isBusy() && rearRightMotor.isBusy() && rearLeftMotor.isBusy() && currentOpmode.opModeIsActive()) {
            waitForTick(50);
        }

        // Stop all motion;
        stopDriveMotors();

        // Turn off RUN_TO_POSITION
        setDriveMotorsRunUsingEncoders();
    }

    public void driveForwardForInches(double inches, double speed) {
        int steps = inchToEncoder(inches);

        driveForwardForSteps(steps, speed);
    }

    public void driveBackwardForInches(double inches, double speed) {
        int steps = inchToEncoder(inches);

        driveBackwardForSteps(steps, speed);
    }

    // Set motors to common speed
    public void setDriveMotorsToCommonSpeed(double speed) {
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        rearLeftMotor.setPower(speed);
        rearRightMotor.setPower(speed);
    }

    // Set the drive motors to run using encoders
    public void setDriveMotorsRunUsingEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Set the drive motors mode to stop and reset encoders
    public void setDriveMotorsResetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Set the drive motors to run to a given position
    public void setDriveMotorsRunToPosition() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Rotate counterclockwise at given speed
    private void rotateCounterClockwise(double speed) {
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setDriveMotorsToCommonSpeed(speed);
    }

    // Rotate clockwise at given speed
    private void rotateClockwise(double speed) {
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveMotorsToCommonSpeed(speed);
    }

    // Rotate counterclockwise by a given amount and at a given speed with using encoders and gyro correction
    public void rotateCounterClockwiseEncoder(int targetDegrees, double speed, Telemetry telemetry) {
        // Adjust the degrees to rotate
        //Undershoot and use the gyro to go the rest of the way
        int degreesToRotate = targetDegrees - 20;

        int numberOfSteps = (4000 / (360 / degreesToRotate));

        rotateCounterClockwise(0.05);
        waitForTick(100);
        stopDriveMotors();

        // Record initial heading
        float initialHeading = getRawGyroHeading();

        // Reset the encoders and set mode
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForTick(50);

        setDriveMotorsRunToPosition();

        waitForTick(50);

        int frontRightMotorStepsToDo = frontRightMotor.getCurrentPosition() - numberOfSteps;
        int frontLeftMotorStepsToDo = frontLeftMotor.getCurrentPosition() - numberOfSteps;
        int rearRightMotorStepsToDo = rearRightMotor.getCurrentPosition() - numberOfSteps;
        int rearLeftMotorStepsToDo = rearLeftMotor.getCurrentPosition() - numberOfSteps;

        // Set target steps
        frontRightMotor.setTargetPosition(frontRightMotorStepsToDo);
        frontLeftMotor.setTargetPosition(frontLeftMotorStepsToDo);
        rearRightMotor.setTargetPosition(rearRightMotorStepsToDo);
        rearLeftMotor.setTargetPosition(rearLeftMotorStepsToDo);

        waitForTick(50);

        setDriveMotorsToCommonSpeed(speed);

        // Keep looping while we are still active, and there is time left, and both motors are running.
        while (frontRightMotor.isBusy() && frontLeftMotor.isBusy() && rearRightMotor.isBusy() && rearLeftMotor.isBusy() && currentOpmode.opModeIsActive()) {
            waitForTick(50);
        }

        // Correct based on the gyro
        float finalHeading = getRawGyroHeading();

        /*
        Since counterclockwise on the gyro is positive, rotating counterclockwise should always increase the heading
        However, if you cross the 180 mark, the sign will flip to a negative value
        The negative value represents the degrees rotated clockwise from the gyro's initial position
        Therefore, the adjusted heading is 360 (which represents a full counterclockwise rotation)
        minus the degrees clockwise from the initial position
        We check to see if finalHeading < initialHeading because that condition will only occur when crossing the 180 mark
         */

        if (finalHeading < initialHeading) {
            finalHeading = 360 - Math.abs(finalHeading);
        }

        float degreesRotated = Math.abs(finalHeading - initialHeading);
        float degreesLeft = targetDegrees - degreesRotated;

        // Loop until the achieved heading is within five degrees of the target
        while (Math.abs(degreesLeft) > 5 && currentOpmode.opModeIsActive()) {
            numberOfSteps = Math.round(4000 / (360 / degreesLeft));

            telemetry.addData("Initial heading", initialHeading);
            telemetry.addData("Final heading", finalHeading);
            telemetry.addData("Degrees rotated", degreesRotated);
            telemetry.addData("Correction", degreesLeft);
            telemetry.addData("Number of steps", numberOfSteps);
            telemetry.update();

            //waitForTick(4000);

            frontRightMotorStepsToDo = frontRightMotor.getCurrentPosition() - numberOfSteps;
            frontLeftMotorStepsToDo = frontLeftMotor.getCurrentPosition() - numberOfSteps;
            rearRightMotorStepsToDo = rearRightMotor.getCurrentPosition() - numberOfSteps;
            rearLeftMotorStepsToDo = rearLeftMotor.getCurrentPosition() - numberOfSteps;

            // Set target steps
            frontRightMotor.setTargetPosition(frontRightMotorStepsToDo);
            frontLeftMotor.setTargetPosition(frontLeftMotorStepsToDo);
            rearRightMotor.setTargetPosition(rearRightMotorStepsToDo);
            rearLeftMotor.setTargetPosition(rearLeftMotorStepsToDo);

            frontRightMotor.setPower(speed);
            frontLeftMotor.setPower(speed);
            rearRightMotor.setPower(speed);
            rearLeftMotor.setPower(speed);

            // Keep looping while we are still active, and there is time left, and both motors are running.
            while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && rearLeftMotor.isBusy() && rearRightMotor.isBusy() && currentOpmode.opModeIsActive()) {
                waitForTick(50);
            }

            finalHeading = getRawGyroHeading();

            if (finalHeading < initialHeading) {
                finalHeading = 360 - Math.abs(finalHeading);
            }

            degreesRotated = Math.abs(finalHeading - initialHeading);
            degreesLeft = targetDegrees - degreesRotated;
        }

        stopDriveMotors();

        setDriveMotorsRunUsingEncoders();
    }

    // Rotate clockwise by a given amount and at a given speed with using encoders and gyro correction
    public void rotateClockwiseEncoder(int targetDegrees, double speed, Telemetry telemetry) {
        // Adjust the degrees to rotate
        // Undershoot and use the gyro to go the rest of the way
        int degreesToRotate = targetDegrees - 20;

        int numberOfSteps = (4000 / (360 / degreesToRotate));

        rotateClockwise(0.05);
        waitForTick(100);
        stopDriveMotors();

        // Record initial heading
        float initialHeading = getRawGyroHeading();

        // Reset the encoders and set mode
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForTick(50);

        setDriveMotorsRunToPosition();

        waitForTick(50);

        int frontRightMotorStepsToDo = frontRightMotor.getCurrentPosition() - numberOfSteps;
        int frontLeftMotorStepsToDo = frontLeftMotor.getCurrentPosition() - numberOfSteps;
        int rearRightMotorStepsToDo = rearRightMotor.getCurrentPosition() - numberOfSteps;
        int rearLeftMotorStepsToDo = rearLeftMotor.getCurrentPosition() - numberOfSteps;

        // Set target steps
        frontRightMotor.setTargetPosition(frontRightMotorStepsToDo);
        frontLeftMotor.setTargetPosition(frontLeftMotorStepsToDo);
        rearRightMotor.setTargetPosition(rearRightMotorStepsToDo);
        rearLeftMotor.setTargetPosition(rearLeftMotorStepsToDo);

        waitForTick(50);

        setDriveMotorsToCommonSpeed(speed);

        // Keep looping while we are still active, and there is time left, and both motors are running.
        while (frontRightMotor.isBusy() && frontLeftMotor.isBusy() && rearRightMotor.isBusy() && rearLeftMotor.isBusy() && currentOpmode.opModeIsActive()) {
            waitForTick(50);
        }

        // Correct based on the gyro
        float finalHeading = getRawGyroHeading();

        /*
        Since clockwise on the gyro is negative, rotating clockwise should always decrease the heading
        However, if you cross the 180 mark, the sign will flip to a positive value
        The positive value represents the degrees rotated counterclockwise from the gyro's initial position
        Therefore, the adjusted heading is negative 360 (which represents a full clockwise rotation)
        plus the degrees counterclockwise from the initial position
        We check to see if finalHeading > initialHeading because that condition will only occur when crossing the 180 mark
         */

        if (finalHeading > initialHeading) {
            finalHeading -= 360;
        }

        float degreesRotated = Math.abs(finalHeading - initialHeading);
        float degreesLeft = targetDegrees - degreesRotated;

        // Loop until the achieved heading is within five degrees of the target
        while (Math.abs(degreesLeft) > 5 && currentOpmode.opModeIsActive()) {
            numberOfSteps = Math.round(4000 / (360 / degreesLeft));

            telemetry.addData("Initial heading", initialHeading);
            telemetry.addData("Final heading", finalHeading);
            telemetry.addData("Degrees rotated", degreesRotated);
            telemetry.addData("Correction", degreesLeft);
            telemetry.addData("Number of steps", numberOfSteps);
            telemetry.update();

            frontRightMotorStepsToDo = frontRightMotor.getCurrentPosition() - numberOfSteps;
            frontLeftMotorStepsToDo = frontLeftMotor.getCurrentPosition() - numberOfSteps;
            rearRightMotorStepsToDo = rearRightMotor.getCurrentPosition() - numberOfSteps;
            rearLeftMotorStepsToDo = rearLeftMotor.getCurrentPosition() - numberOfSteps;

            // Set target steps
            frontRightMotor.setTargetPosition(frontRightMotorStepsToDo);
            frontLeftMotor.setTargetPosition(frontLeftMotorStepsToDo);
            rearRightMotor.setTargetPosition(rearRightMotorStepsToDo);
            rearLeftMotor.setTargetPosition(rearLeftMotorStepsToDo);

            frontRightMotor.setPower(speed);
            frontLeftMotor.setPower(speed);
            rearRightMotor.setPower(speed);
            rearLeftMotor.setPower(speed);

            // Keep looping while we are still active, and there is time left, and both motors are running.
            while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && rearLeftMotor.isBusy() && rearRightMotor.isBusy() && currentOpmode.opModeIsActive()) {
                waitForTick(50);
            }

            finalHeading = getRawGyroHeading();

            if (finalHeading > initialHeading) {
                finalHeading -= 360;
            }

            degreesRotated = Math.abs(finalHeading - initialHeading);
            degreesLeft = targetDegrees - degreesRotated;
        }

        stopDriveMotors();

        setDriveMotorsRunUsingEncoders();
    }

    // Strafe left with encoders and gyro correction
    public void autoStrafeLeft(int numberOfSteps, double speed)// KT: Made lots of changes!
    {
        // Set the motor directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForTick(40);

        // Define step counts
        int frontRightMotorStepsToDo = frontRightMotor.getCurrentPosition() + numberOfSteps;
        int frontLeftMotorStepsToDo = frontLeftMotor.getCurrentPosition() + numberOfSteps;
        int rearRightMotorStepsToDo = rearRightMotor.getCurrentPosition() + numberOfSteps;
        int rearLeftMotorStepsToDo = rearLeftMotor.getCurrentPosition() + numberOfSteps;

        waitForTick(40);

        // Set the target steps
        frontRightMotor.setTargetPosition(frontRightMotorStepsToDo);
        frontLeftMotor.setTargetPosition(frontLeftMotorStepsToDo);
        rearRightMotor.setTargetPosition(rearRightMotorStepsToDo);
        rearLeftMotor.setTargetPosition(rearLeftMotorStepsToDo);

        waitForTick(40);

        // Turn on run to position
        setDriveMotorsRunToPosition();

        waitForTick(40);

        // Start motion.
        setDriveMotorsToCommonSpeed(Math.abs(speed));

        // Keep looping while we are still active, and there is time left, and both motors are running.
        while (frontRightMotor.isBusy() && frontLeftMotor.isBusy() && rearRightMotor.isBusy() && rearLeftMotor.isBusy() && currentOpmode.opModeIsActive()) {
            waitForTick(50);
        }

        // Stop all motion
        setDriveMotorsToCommonSpeed(0);

        waitForTick(40);

        // Turn off run to position
        setDriveMotorsRunUsingEncoders();
    }

    // Strafe right with encoders and gyro correction
    public void autoStrafeRight(int numberOfSteps, double speed)// KT: Made lots of changes!
    {
        // Set the motor directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForTick(40);

        // Define step counts
        int frontRightMotorStepsToDo = frontRightMotor.getCurrentPosition() + numberOfSteps;
        int frontLeftMotorStepsToDo = frontLeftMotor.getCurrentPosition() + numberOfSteps;
        int rearRightMotorStepsToDo = rearRightMotor.getCurrentPosition() + numberOfSteps;
        int rearLeftMotorStepsToDo = rearLeftMotor.getCurrentPosition() + numberOfSteps;

        waitForTick(40);

        // Set the target steps
        frontRightMotor.setTargetPosition(frontRightMotorStepsToDo);
        frontLeftMotor.setTargetPosition(frontLeftMotorStepsToDo);
        rearRightMotor.setTargetPosition(rearRightMotorStepsToDo);
        rearLeftMotor.setTargetPosition(rearLeftMotorStepsToDo);

        waitForTick(40);

        // Turn on run to position
        setDriveMotorsRunToPosition();

        waitForTick(40);

        // Start motion.
        setDriveMotorsToCommonSpeed(Math.abs(speed));

        // Keep looping while we are still active, and there is time left, and both motors are running.
        while (frontRightMotor.isBusy() && frontLeftMotor.isBusy() && rearRightMotor.isBusy() && rearLeftMotor.isBusy() && currentOpmode.opModeIsActive()) {
            waitForTick(50);
        }

        // Stop all motion
        setDriveMotorsToCommonSpeed(0);

        waitForTick(40);

        // Turn off run to position
        setDriveMotorsRunUsingEncoders();
    }

    // Lower the lift
    public void lowerLiftMotor(double power) {
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setPower(power);
    }

    // Raise the lift
    public void raiseLiftMotor(double power) {
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setPower(power);
    }

    // Stop the lift
    public void stopLiftMotor() {
        liftMotor.setPower(0);
    }

    // Move the crane backward
    public void craneMotorBackward() {
        craneExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        craneExtensionMotor.setPower(Constants.CRANE_SPEED);
    }

    // Move the crane forward
    public void craneMotorForward() {
        craneExtensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        craneExtensionMotor.setPower(Constants.CRANE_SPEED);
    }

    // Stop the crane motor
    public void stopCraneMotor() {
        craneExtensionMotor.setPower(0);
    }

    // Lower jewel arm
    public void lowerJewelArm() {
        jewelLiftServo.setPosition(0.7);
        waitForTick(400);
        jewelLiftServo.setPosition(0.5);
        waitForTick(400);
        jewelLiftServo.setPosition(0.4);
    }

    public void jewelFlipLeft() {
        jewelFlipServo.setPosition(0);
    }

    public void jewelFlipRight() {
        jewelFlipServo.setPosition(1);
    }

    public void resetJewelServos() {
        jewelLiftServo.setPosition(0.9);
        jewelFlipServo.setPosition(0.5);
    }

    // Close the clamp servos
    public void closeClampServos() {
        clampServoLeft.setPosition(0.71);
        clampServoRight.setPosition(0.24);
    }

    // Open the clamp servos halfway for isolating one block
    public void openClampServosHalfway() {
        clampServoLeft.setPosition(0.58);
        clampServoRight.setPosition(0.37);
    }

    // Open the clamp servos
    public void openClampServos() {
        clampServoLeft.setPosition(0.30);
        clampServoRight.setPosition(0.65);
    }

    // Move the crane servo left
    public void craneRotateLeft() {
        craneRotateServo.setPower(-0.1);
    }

    // Move the crane servo right
    public void craneRotateRight() {
        craneRotateServo.setPower(0.1);
    }

    // Stop the crane servo
    public void craneRotateStop() {
        craneRotateServo.setPower(0);
    }

    // Move the wrist up
    public void clawWristDown() {
        clawWristServo.setPosition(0);
    }

    // Put the claw wrist in a lower than halfway position
    public void clawWristLow() {
        clawWristServo.setPosition(0.12);
    }

    // Put the claw wrist in a halfway position
    public void clawWristHalfway() {
        clawWristServo.setPosition(0.18);
    }

    // Move the wrist down
    public void clawWristUp() {
        clawWristServo.setPosition(0.5);
    }

    // Close the claw
    public void clawPinch() {
        clawPinchServo.setPosition(0.6);
    }

    // Pinch halfway
    public void clawPinchHalfway() {
        clawPinchServo.setPosition(0.8);
    }

    // Open the claw
    public void clawOpen() {
        clawPinchServo.setPosition(1);
    }

    // Get the raw gyro heading
    public float getRawGyroHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    // Get the gyro heading with counterclockwise as positive
    public float getCounterClockwiseGyroHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float rawAngle = angles.firstAngle;
        /*
        The gyro returns clockwise values as negative from 0 to -180, and counterclockwise as 0 to 180
        Therefore, we have to convert to standard 0-360 values with counterclockwise as positive
         */
        float convertedAngle;

        if (rawAngle < 0) {
            convertedAngle = 360 + rawAngle;
        } else {
            convertedAngle = rawAngle;
        }

        return convertedAngle;
    }

    // Get the gyro heading with clockwise ans positive
    public float getClockwiseGyroHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float rawAngle = angles.firstAngle;
        /*
        The gyro returns clockwise values as negative from 0 to -180, and counterclockwise as 0 to 180
        Therefore, we have to convert to standard 0-360 values with clockwise as positive
         */
        float convertedAngle;

        if (rawAngle < 0) {
            convertedAngle = rawAngle * -1;
        } else {
            convertedAngle = 360 - rawAngle;
        }

        return convertedAngle;
    }

    // Return the range sensor's distance (in inches)
    public double getRangeSensorDistance() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    // Displays RGB info from the color sensors
    public void displayColorValues(Telemetry telemetry) {
        double roundedLeftHue = Math.round(getLeftSensorHue() * 10) / 10;
        double roundedRightHue = Math.round(getRightSensorHue() * 10) / 10;

        telemetry.addLine("Left Sensor")
                .addData("Red", leftColorSensor.red())
                .addData("Green", leftColorSensor.green())
                .addData("Blue", leftColorSensor.blue())
                .addData("Hue", roundedLeftHue);

        telemetry.addLine("Right Sensor")
                .addData("Red", rightColorSensor.red())
                .addData("Green", rightColorSensor.green())
                .addData("Blue", rightColorSensor.blue())
                .addData("Hue", roundedRightHue);
    }

    // Get hue from the left sensor
    public double getLeftSensorHue() {
        // Get the RGB values from the sensor and convert to HSV
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV((leftColorSensor.red() * 255) / 800, (leftColorSensor.green() * 255) / 800, (leftColorSensor.blue() * 255) / 800, hsvValues);

        // Retrieve the Hue value
        double currentHue = hsvValues[0];

        return currentHue;
    }

    // Get hue from the right sensor
    public double getRightSensorHue() {
        // Get the RGB values from the sensor and convert to HSV
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV((rightColorSensor.red() * 255) / 800, (rightColorSensor.green() * 255) / 800, (rightColorSensor.blue() * 255) / 800, hsvValues);

        // Retrieve the Hue value
        double currentHue = hsvValues[0];

        return currentHue;
    }

    // Turn the LEDs off
    public void turnOffLEDs() {
        leftLED.setState(false);
        rightLED.setState(false);
    }

    // Turn the LEDs on
    public void turnOnLEDs() {
        leftLED.setState(true);
        rightLED.setState(true);
    }

    // Convert from inches to encoder counts
    public int inchToEncoder(double inches) {
        int ticks = (int)Math.round(inches * 39); // 39 encoder ticks for one inch

        return ticks;
    }

}
