//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;


/**
 * This is NOT an opmode.
 *
 * This class defines all the specific hardware for the Edge robot.
 *1-11-2017 updated to accomodate the adafruit color sensor
 */
public class theEdge {
    // Create a timer
    private ElapsedTime timer = new ElapsedTime();

    // What do we have connected to the robot???
    // Motors
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor rearLeftMotor = null;
    public DcMotor rearRightMotor = null;
    public DcMotor ballLift = null;
    public DcMotor particleShooter = null;
    public DcMotor particleLift = null;
    public DcMotor particleShift = null;//KT

    // Servo's
    public CRServo ballPull = null;
    public Servo particleLoad = null;
    public Servo beaconHitterR = null;//KT
    public Servo beaconHitterL = null;//KT


    // Sensors
    public ModernRoboticsI2cGyro gyro = null;
    //    public ColorSensor colorDetector = null;
    public ColorSensor sensorRGB = null;
    public ColorSensor sensorRGB2 = null;
    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int LED_CHANNEL = 5;

    public DeviceInterfaceModule cdim = null;


    public ModernRoboticsI2cRangeSensor rangeSensor = null;
    public TouchSensor wallButton = null;
    public TouchSensor registrationButton = null;
    public OpticalDistanceSensor lineTracker = null;

    // test
    public TouchSensor launcherPositionSensor = null;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public theEdge() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Setup the continuous Rotation Servo
        ballPull = hwMap.crservo.get("ball_pull");

        // Setup the normal Servo's
        particleLoad = hwMap.servo.get("particle_advance");
        beaconHitterR = hwMap.servo.get("beacon_hitter_R");//KT: Added
        beaconHitterL = hwMap.servo.get("beacon_hitter_L");//KT: Added


        // Initialize the continuous rotation servo
        ballPull.setDirection(DcMotorSimple.Direction.FORWARD);
        ballPull.setPower(0);

        // Initialize the normal servo's
        particleLoad.setPosition(0.3); //KT: was 0
        beaconHitterR.setPosition(1); // KT: New
        beaconHitterL.setPosition(1); // KT: New


        // Define and Initialize Motors
        frontLeftMotor = hwMap.dcMotor.get("front_left_motor");
        frontRightMotor = hwMap.dcMotor.get("front_right_motor");
        rearLeftMotor = hwMap.dcMotor.get("rear_left_motor");
        rearRightMotor = hwMap.dcMotor.get("rear_right_motor");
        ballLift = hwMap.dcMotor.get("ball_lift");
        particleShooter = hwMap.dcMotor.get("particle_shooter");
        particleLift = hwMap.dcMotor.get("particle_lift");
        particleShift = hwMap.dcMotor.get("particle_shift");//KT:


        ballLift.setDirection(DcMotor.Direction.FORWARD);
        particleShooter.setDirection(DcMotor.Direction.REVERSE);
        particleLift.setDirection(DcMotor.Direction.FORWARD);
        particleShift.setDirection(DcMotor.Direction.REVERSE);

        // Set the motor mode (with or without encoders)
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        particleShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        particleLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        particleShift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        ballLift.setPower(0);
        particleShooter.setPower(0);
        particleLift.setPower(0);
        particleShift.setPower(0);

        // Define and initialize sensors
        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");
//        colorDetector = hwMap.colorSensor.get("color");
        // get a reference to our DeviceInterfaceModule object.
        cdim = hwMap.deviceInterfaceModule.get("dim"); // These two line are needed to enable the LED for the color sensor - now disabled
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        sensorRGB = hwMap.colorSensor.get("color");
        sensorRGB2 = hwMap.colorSensor.get("color2");
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor");
        wallButton = hwMap.touchSensor.get("wall_button");
        registrationButton = hwMap.touchSensor.get("registration_button");
        lineTracker = hwMap.opticalDistanceSensor.get("line_track");

        // Test
        launcherPositionSensor = hwMap.touchSensor.get("shooter_position");

        // turn the LED on the light sensor off
        lineTracker.enableLed(false);
    }

    /****************************************************************************************************************************************
     * /************************************************************************************************************************************
     * <p>
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    //********Color Sensor 1**********************************************
    public double getRGB1HSV(Telemetry telemetry) {
        // Get the color from the sensor
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
        double currentHSV = hsvValues[0];
        if ((currentHSV == 0)&& ((sensorRGB.red()> sensorRGB.green())|| (sensorRGB.red()> sensorRGB.green()))){
            currentHSV = 0.01;
        }
        waitForTick(20);

        // telemetry.addData("Sensor", "1");
        //telemetry.addData("Clear", sensorRGB.alpha());
        //telemetry.addData("Red  ", sensorRGB.red());
        //telemetry.addData("Green", sensorRGB.green());
        //telemetry.addData("Blue ", sensorRGB.blue());
        //telemetry.addData("Hue", hsvValues[0]);
        //telemetry.update();

        return currentHSV;
    }
//**********Color Sensor 2 *********************************************

    public double getRGB2HSV(Telemetry telemetry) {
        // Get the color from the sensor
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV((sensorRGB2.red() * 255) / 800, (sensorRGB2.green() * 255) / 800, (sensorRGB2.blue() * 255) / 800, hsvValues);
        double currentHSV = hsvValues[0];
        if ((currentHSV == 0)&& ((sensorRGB2.red()> sensorRGB2.green())|| (sensorRGB2.red()> sensorRGB2.green()))){
            currentHSV = 0.01;
        }

        waitForTick(20);
        //  telemetry.addData("Sensor", "2");
        // telemetry.addData("Clear", sensorRGB2.alpha());
        //telemetry.addData("Red  ", sensorRGB2.red());
        //telemetry.addData("Green", sensorRGB2.green());
        //telemetry.addData("Blue ", sensorRGB2.blue());
        //telemetry.addData("Hue", hsvValues[0]);
        //telemetry.update();

        return currentHSV;
    }


    //************Calibrate Gyro **********************************************
    public void calibrateGyro() {
        // Send telemetry message to alert driver that we are calibrating;
//        telemetry.addData(">", "Calibrating Gyro");    //
//        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating()) {
            waitForTick(50);
//            sleep(50);
//            idle();
        }
        if(gyro.getHeading() != 0) {
            calibrateGyro();
        }
//        telemetry.addData(">", "Robot Ready.");    //
//        telemetry.update();
    }

    //************************************* Particle Shoot **********************************************************
    public void particleShootByEncoder(int numberOfSteps2) {
       // particleShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //try to see if this fixes the loop issue
        waitForTick(5); // Was 100

        int stepsToDo = particleShooter.getCurrentPosition() + numberOfSteps2;
        particleShooter.setTargetPosition(stepsToDo);
        particleShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //telemetry.addData("Shooting encoder... ","%.2f", stepsToDo());
        // particleShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // particleShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForTick(40);
        //     runtime.reset();

        particleShooter.setPower(1);// KT:Was  1.0
        // double timeout = 3.0; // 2 second timeout

        boolean loadNextParticleFlag = false;

        while (particleShooter.isBusy()) {
            // Wait until half of the steps are complete
            if (particleShooter.getCurrentPosition() > (stepsToDo - (numberOfSteps2 / 2))) {
                loadNextParticleFlag = true;
            }
            loadNextParticle(loadNextParticleFlag);


            // sit and wait for them to finish
//           telemetry.addData("Shooting Particle... ","%.2f", runtime.seconds());
//           telemetry.update();
            // waitForTick(40);
        }
        // particleShooter.setPower(0.0);
     //   particleShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // see if this turns motor off
        loadNextParticle(false);  // Reset it
    }

    //************************************* Particle Shoot 1 **** Use to Jog Launcher ******************************************************
    public void particleShootByEncoder1(int numberOfSteps1) {
      //  particleShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //try to see if this fixes the loop issue
        waitForTick(5); // Was 100

        int stepsToDo = particleShooter.getCurrentPosition() + numberOfSteps1;
        particleShooter.setTargetPosition(stepsToDo);
        particleShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //telemetry.addData("Shooting encoder... ","%.2f", stepsToDo());
        // particleShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // particleShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForTick(40);
        //     runtime.reset();
        particleShooter.setPower(1);// KT:Was  .7
        // double timeout = 3.0; // 2 second timeout

        while (particleShooter.isBusy()) {
            // sit and wait for them to finish
            //           telemetry.addData("Shooting Particle... ","%.2f", runtime.seconds());
            //           telemetry.update();
            // waitForTick(40);
        }
        // particleShooter.setPower(0.0);
      //  particleShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // see if this turns motor off
    }


    // **************************************
    public void allWheelsStop() {
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
    }

    //****************************************
    public void setDriveMotorsUsingEncoders() {
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //****************************************KT New
    public void setDriveMotorsResetEncoders() {
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //****************************************
    public void setDriveMotorsRunToPosition() {
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //****************************************
    public void setDriveMotorsToCommonSpeed(double speed) {
        frontRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        rearRightMotor.setPower(speed);
        rearLeftMotor.setPower(speed);
    }

    //*****************************************
    public void moveForward(double speed) {
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveMotorsUsingEncoders();
        setDriveMotorsToCommonSpeed(speed);
    }

    //*****************************************
    public void moveBackward(double speed) {
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setDriveMotorsUsingEncoders();
        setDriveMotorsToCommonSpeed(speed);
    }

    //******************************************strafe Right *****************
    public void strafeRight(double speed) {
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setDriveMotorsUsingEncoders();
        setDriveMotorsToCommonSpeed(speed);
    }

    //******************************************
    public void strafeLeft(double speed) {
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveMotorsUsingEncoders();
        setDriveMotorsToCommonSpeed(speed);
    }


    public void rotateToDirectionCloseEnough(Telemetry telemetry, double desiredAngle, double tolerance) {
        // define a couple of parameters to adjust the sensitivity of the seeking
        double reduceSpeedThreshold = 20;
        double highSpeed = 0.7; //KT: was 0.4
        double lowSpeed = 0.2;  //KT: was 0.05

        // deal with negative target numbers
        if (desiredAngle < 0) {
            desiredAngle += 360;
        }

        boolean rotatingToTarget = true;
        while (rotatingToTarget) {
            double currentDirection = getCurrentHeading();

            // determine if it is closer to turn clockwise or counter clockwise
            telemetry.addData("Inside Target Heading:   ", "%.2f", desiredAngle);
            telemetry.addData("Inside Current Heading:  ", "%.2f", currentDirection);

            double delta = desiredAngle - currentDirection;

            if (Math.abs(delta) < tolerance) {
                rotatingToTarget = false;
            }

            if (delta < 0) {
                delta += 360;
            }
            if (delta == 0) {
                rotatingToTarget = false;
            } else if (delta <= 180) {
                // delta approaches 0 as clockwise rotation nears target angle
                if (delta > reduceSpeedThreshold) {
                    rotateClockwise(highSpeed);
                    telemetry.addData("Rotation Speed", "CW Fast");
                } else {
                    rotateClockwise(lowSpeed);
                    telemetry.addData("Rotation Speed", "CW Slow");
                }
            } else {
                // delta approaches 360 as counter clockwise rotation nears target angle
                if (360 - delta > reduceSpeedThreshold) {
                    rotateCounterClockwise(highSpeed);
                    telemetry.addData("Rotation Speed", "CCW Fast");
                } else {
                    rotateCounterClockwise(lowSpeed);
                    telemetry.addData("Rotation Speed", "CCW Slow");
                }
            }
            telemetry.addData("Target Heading:   ", "%.2f", desiredAngle);
            telemetry.addData("Current Heading:  ", "%.2f", currentDirection);
            telemetry.addData("Delta:       ", "%.2f", delta);
            telemetry.update();
        }
        allWheelsStop();
    }


    //******************************************
    public void rotateToDirection(Telemetry telemetry, double desiredAngle, double reduceSpeedThreshold, double highSpeed, double lowSpeed) {
        // define a couple of parameters to adjust the sensitivity of the seeking
//        double reduceSpeedThreshold = 20;
//        double highSpeed = 0.7; //KT: was 0.4
//        double lowSpeed = 0.1;  //KT: was 0.05

        // deal with negative target numbers
        if (desiredAngle < 0) {
            desiredAngle += 360;
        }

        boolean rotatingToTarget = true;
        while (rotatingToTarget) {
            double currentDirection = getCurrentHeading();

            // determine if it is closer to turn clockwise or counter clockwise
            telemetry.addData("Inside Target Heading:   ", "%.2f", desiredAngle);
            telemetry.addData("Inside Current Heading:  ", "%.2f", currentDirection);

            double delta = desiredAngle - currentDirection;

            if (delta < 0) {
                delta += 360;
            }
            if (delta == 0) {
                rotatingToTarget = false;
            } else if (delta <= 180) {
                // delta approaches 0 as clockwise rotation nears target angle
                if (delta > reduceSpeedThreshold) {
                    rotateClockwise(highSpeed);
                    telemetry.addData("Rotation Speed", "CW Fast");
                } else {
                    rotateClockwise(lowSpeed);
                    telemetry.addData("Rotation Speed", "CW Slow");
                }
            } else {
                // delta approaches 360 as counter clockwise rotation nears target angle
                if (360 - delta > reduceSpeedThreshold) {
                    rotateCounterClockwise(highSpeed);
                    telemetry.addData("Rotation Speed", "CCW Fast");
                } else {
                    rotateCounterClockwise(lowSpeed);
                    telemetry.addData("Rotation Speed", "CCW Slow");
                }
            }
            telemetry.addData("Target Heading:   ", "%.2f", desiredAngle);
            telemetry.addData("Current Heading:  ", "%.2f", currentDirection);
            telemetry.addData("Delta:       ", "%.2f", delta);
            telemetry.update();
        }
        allWheelsStop();
    }

    //****************************************************
    public void rotateClockwise(double speed) {


        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        rearRightMotor.setPower(speed);
        rearLeftMotor.setPower(speed);
    }

    //**************************************************** KT New
    public void rotateClockwiseEncoder(int degreesToRotate, double speed) {

       // double startPosition = getCurrentHeading(); // get start heading
        int numberOfSteps = (8000 / (360 / degreesToRotate));
        // Take up slack before move need to play with time and speed


        rotateClockwise(.05);
        waitForTick(100);
        allWheelsStop(); // just added

        // Now reset the encoders and set mode

       // setDriveMotorsUsingEncoders(); 4-27
        waitForTick(10);
        //   setDriveMotorsResetEncoders(); // new KT
        //   waitForTick(10);
        setDriveMotorsRunToPosition();
        waitForTick(15);
        int frontRightMotorStepsToDo = frontRightMotor.getCurrentPosition() + numberOfSteps;
        int frontLeftMotorStepsToDo = frontLeftMotor.getCurrentPosition() + numberOfSteps;
        int rearRightMotorStepsToDo = rearRightMotor.getCurrentPosition() + numberOfSteps;
        int rearLeftMotorStepsToDo = rearLeftMotor.getCurrentPosition() + numberOfSteps;

        // set the target steps
        frontRightMotor.setTargetPosition(frontRightMotorStepsToDo);
        frontLeftMotor.setTargetPosition(frontLeftMotorStepsToDo);
        rearRightMotor.setTargetPosition(rearRightMotorStepsToDo);
        rearLeftMotor.setTargetPosition(rearLeftMotorStepsToDo);

        waitForTick(10);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        rearRightMotor.setPower(speed);
        rearLeftMotor.setPower(speed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (frontRightMotor.isBusy() &&
                frontLeftMotor.isBusy() &&
                rearRightMotor.isBusy() &&
                rearLeftMotor.isBusy()) {

            waitForTick(50);
        }

        allWheelsStop(); // just added

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //**************************************************** KT New
    public void rotateCounterClockwiseEncoder(int degreesToRotate, double speed) {
        int numberOfSteps = (8000 / (360 / degreesToRotate));
        // Take up slack before move need to play with time and speed
       // double startPosition = getCurrentHeading(); // get start heading

      rotateCounterClockwise(.05);
       waitForTick(100);
        allWheelsStop(); // just added
    //waitForTick(2000);
        // Now reset the encoders and set mode
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD); //4-27 was below and reverse
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
      //  setDriveMotorsUsingEncoders(); // removed 4-27
        waitForTick(50);
        //   setDriveMotorsResetEncoders(); // new KT
        //   waitForTick(10);
        setDriveMotorsRunToPosition();
        waitForTick(50);
        int frontRightMotorStepsToDo = frontRightMotor.getCurrentPosition() - numberOfSteps;//4-27 was +
        int frontLeftMotorStepsToDo = frontLeftMotor.getCurrentPosition() - numberOfSteps;
        int rearRightMotorStepsToDo = rearRightMotor.getCurrentPosition() - numberOfSteps;
        int rearLeftMotorStepsToDo = rearLeftMotor.getCurrentPosition() - numberOfSteps;

        // set the target steps
        frontRightMotor.setTargetPosition(frontRightMotorStepsToDo);
        frontLeftMotor.setTargetPosition(frontLeftMotorStepsToDo);
        rearRightMotor.setTargetPosition(rearRightMotorStepsToDo);
        rearLeftMotor.setTargetPosition(rearLeftMotorStepsToDo);

        waitForTick(50); //was 10 4-27
//        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        rearLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

      //  frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      //  rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      //  rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
waitForTick(50); //just added 4-27
        frontRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        rearRightMotor.setPower(speed);
        rearLeftMotor.setPower(speed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        while ( frontRightMotor.isBusy() &&
                frontLeftMotor.isBusy() &&
                rearRightMotor.isBusy() &&
                rearLeftMotor.isBusy()) {

            waitForTick(50);
        }

        allWheelsStop(); // just added
//setDriveMotorsUsingEncoders(); set back mode
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
        //********************** HS Rotation Correction *****************************************
    // This function splits the difference
    public void rotateByEncoderCorrection(int desiredHeading, double speed, int degreeError) {
     //  waitForTick(150);
        // to make sure its stable before using this number to calculate correction
        double startHeading = getCurrentHeading();
        if ((startHeading < desiredHeading )&& ((desiredHeading-startHeading)> degreeError) && ((desiredHeading-startHeading)>0)) {
            rotateClockwiseEncoder((int)(desiredHeading - startHeading), speed);
            allWheelsStop();
        }

       if((startHeading > desiredHeading )&& ((startHeading-desiredHeading)>degreeError) && ((startHeading-desiredHeading)>0))  {
               rotateCounterClockwiseEncoder((int)(startHeading-desiredHeading), speed);
           allWheelsStop();
       }

    }




    //***************************************************
    public void rotateCounterClockwise(double speed) {
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        rearRightMotor.setPower(speed);
        rearLeftMotor.setPower(speed);
    }

    // Because I'm not sure of the exact encoders being used, this just ueses steps.
    // You can add some math and constants (like the ones commented out at the top) to
    // allow you to specify the distance in inches or cm.
    // Speed should be between 0.0 and 1.0
    // Timout is number of seconds before it will stop it, this is a safeguard
    // specify seconds in x.x
    // ex autoDriveForward(16000, 0.5, 5.0);
    //********************************************  Auto Drive Forward ***********************************
    public void autoDriveForward(int numberOfSteps, double speed) {
        // Setup motor directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // define the step counts
        int frontRightMotorStepsToDo = frontRightMotor.getCurrentPosition() + numberOfSteps;
        int frontLeftMotorStepsToDo = frontLeftMotor.getCurrentPosition() + numberOfSteps; // was -
        int rearRightMotorStepsToDo = rearRightMotor.getCurrentPosition() + numberOfSteps; // was -
        int rearLeftMotorStepsToDo = rearLeftMotor.getCurrentPosition() + numberOfSteps;

        // set the target steps
        frontRightMotor.setTargetPosition(frontRightMotorStepsToDo);
        frontLeftMotor.setTargetPosition(frontLeftMotorStepsToDo);
        rearRightMotor.setTargetPosition(rearRightMotorStepsToDo);
        rearLeftMotor.setTargetPosition(rearLeftMotorStepsToDo);

        // Turn On RUN_TO_POSITION
        setDriveMotorsRunToPosition();

        // start motion
        setDriveMotorsToCommonSpeed(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (frontRightMotor.isBusy() &&
                frontLeftMotor.isBusy() &&
                rearRightMotor.isBusy() &&
                rearLeftMotor.isBusy()) {
            waitForTick(50);
        }

        // Stop all motion;
        allWheelsStop();

        // Turn off RUN_TO_POSITION
        setDriveMotorsUsingEncoders();

    }
//********************************************** Auto Drive Backward  *****************************
    public void autoDriveBackward(int numberOfSteps, double speed) {
        // set wheel directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // define the step counts
        int frontRightMotorStepsToDo = frontRightMotor.getCurrentPosition() + numberOfSteps; // was -
        int frontLeftMotorStepsToDo = frontLeftMotor.getCurrentPosition() + numberOfSteps;
        int rearRightMotorStepsToDo = rearRightMotor.getCurrentPosition() + numberOfSteps;
        int rearLeftMotorStepsToDo = rearLeftMotor.getCurrentPosition() + numberOfSteps; // was -

        // set the target steps
        frontRightMotor.setTargetPosition(frontRightMotorStepsToDo);
        frontLeftMotor.setTargetPosition(frontLeftMotorStepsToDo);
        rearRightMotor.setTargetPosition(rearRightMotorStepsToDo);
        rearLeftMotor.setTargetPosition(rearLeftMotorStepsToDo);

        // Turn On RUN_TO_POSITION
        setDriveMotorsRunToPosition();

        // start motion
        setDriveMotorsToCommonSpeed(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (frontRightMotor.isBusy() &&
                frontLeftMotor.isBusy() &&
                rearRightMotor.isBusy() &&
                rearLeftMotor.isBusy()) {
            waitForTick(50);
        }

        // Stop all motion;
        allWheelsStop();

        // Turn off RUN_TO_POSITION
        setDriveMotorsUsingEncoders();

    }
//*************************************************
    public double getCurrentHeading() {


        /*
        double heading = gyro.getIntegratedZValue();
        heading %= 360; // Make sure we are within 0 to 360
        if (heading < 0) { // Make sure we only return positive values
            heading += 360;
        }
//        heading = heading - 360 * -1;
*/
        double heading = gyro.getHeading();
        return heading;
    }
//****************************************************
    public void runParticleLift(double secondsToRun)
    {
        timer.reset();

        // keep looping while we there is time left
        while (timer.seconds() < secondsToRun)
        {
            particleLift.setPower(1.0);
//            telemetry.addData("Running Particle Lift... ","%.2f", secondsToRun - runtime.seconds());
//            telemetry.update();
            waitForTick(40);
        }
        particleLift.setPower(0.0);
    }

//*******************************************************
    public void runParticleShooter(double secondsToRun)
    {
        timer.reset();

        // keep looping while we there is time left
        while (timer.seconds() < secondsToRun)
        {
            particleShooter.setPower(-1.0);
//            telemetry.addData("Running Particle Shooter... ","%.2f", secondsToRun - runtime.seconds());
//            telemetry.update();
            waitForTick(40);
        }
        particleShooter.setPower(0.0);
    }
//*****************************************************
    public void runBallLift(double secondsToRun)
    {
        timer.reset();
        ballLift.setDirection(DcMotor.Direction.FORWARD);
        // keep looping while we there is time left
        while (timer.seconds() < secondsToRun)
        {
            ballLift.setPower(1.0);
//            telemetry.addData("Running Ball Lift... ","%.2f", secondsToRun - runtime.seconds());
//            telemetry.update();
            waitForTick(40);
        }
        ballLift.setPower(0.0);
    }
    //****************************************************
    public void runBallLower(double secondsToRun)
    {
        timer.reset();
        ballLift.setDirection(DcMotor.Direction.REVERSE);
        // keep looping while we there is time left
        while (timer.seconds() < secondsToRun)
        {
//            ballLift.setDirection(mot)
            ballLift.setPower(1.0);
//            telemetry.addData("Running Ball Lift... ","%.2f", secondsToRun - runtime.seconds());
//            telemetry.update();
            waitForTick(40);
        }
        ballLift.setPower(0.0);
    }
//*******************************************************
    public boolean getLauncherPosition(){
        if(launcherPositionSensor.getValue() > 0) {
            return true;
        }
        else {
            return false;
        }
    }
    //**********************************************
    // Returns true if it is on a line
    public boolean lineTrack(Telemetry telemetry) {
        double threshold = 0.5;// was 0.8
        lineTracker.enableLed(true);
        double rawValue = lineTracker.getLightDetected();
        telemetry.addData("Light Level", "%.2f", rawValue);
        telemetry.update();
        if(rawValue > threshold){
            return true;
        }
        return false;
    }
//************************************************** KT  CHANGED
    public double rangeSensor(Telemetry telemetry) {
        telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", rangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
        double realDist = rangeSensor.getDistance(DistanceUnit.CM);
        return realDist;

    }
//************************************ Load next particle *******************
    public void loadNextParticle(boolean active) {
//        double currentPosition = particleLoad.getPosition();
//        telemetry.addData("current position", "%.2f", currentPosition);
//        telemetry.update();
//        waitForTick(3000);
        double stopPosition = 0;  // valid values are between 0.0 - 1.0  KT: 3-4-2017 changes variables
        double homePosition = 0.3;  // valid values are between 0.0 - 1.0
        if(active) {
            particleLoad.setPosition(stopPosition);
        }
        else {
            particleLoad.setPosition(homePosition);
        }
//        waitForTick(3000);

    }
    //************************************ Flip Right Beacon  *******************
    public void setBeaconHitterR(boolean active) {
//        double currentPosition = beaconHitterR.getPosition();
//        telemetry.addData("current position", "%.2f", currentPosition);
//        telemetry.update();
//        waitForTick(3000);
        double stopPosition = 0;  // valid values are between 0.0 - 1.0  KT: 3-4-2017 changes variables
        double homePosition = 1;  // valid values are between 0.0 - 1.0
        if(active) {
            beaconHitterR.setPosition(stopPosition);
        }
        else {
            beaconHitterR.setPosition(homePosition);
        }
//        waitForTick(3000);

    }

    //************************************ Flip Left Beacon  *******************
    public void setBeaconHitterL(boolean active) {
//        double currentPosition = beaconHitterL.getPosition();
//        telemetry.addData("current position", "%.2f", currentPosition);
//        telemetry.update();
//        waitForTick(3000);
        double stopPosition = .2;  // valid values are between 0.0 - 1.0  KT: 3-4-2017 changes variables
        double homePosition = 1;  // valid values are between 0.0 - 1.0
        if(active) {
            beaconHitterL.setPosition(stopPosition);
        }
        else {
            beaconHitterL.setPosition(homePosition);
        }
//        waitForTick(3000);

    }
//*********************************************
    public double getControllerAngle(double yStick, double xStick) {

        double controllerAngle = -1;
        // don't allow an undefined angle
        if ((yStick != 0) || (xStick != 0)) {
            // get inverse tangent in radians
            // convert to degrees using 180/PI
            // This will result in the following:
            // (0,1) = 180, (1,-1) = 45, (1,0) = 90, (1,1) = 135
            // (-1,1) = -135, (-1,0) = -90, (-1,-1) = -45
            controllerAngle = 90 - Math.atan2((-1 * yStick), xStick) * (180 / Math.PI);

            // If the value is less than 0, add 360 to make it positive
            if (controllerAngle < 0) {
                controllerAngle += 360;
            }
        }
        return controllerAngle;
    }
//****************************************************
    public double getControllerAngleDeadband(double yStick, double xStick, double tolerance) {

        double controllerAngle = getControllerAngle(yStick, xStick);

        // Run the resulting angle through a deadband filter to make it easier to drive
        // in straight lines.  We care about 0, 90, 180, 360
        if(tolerance > 90){
            return controllerAngle;
        }
        else if((controllerAngle <= (0 + tolerance)) && (controllerAngle >= (360 - tolerance))){
            controllerAngle = 0;
        }
        else if((controllerAngle <= (90 + tolerance)) && (controllerAngle >= (90 - tolerance))){
            controllerAngle = 90;
        }
        else if((controllerAngle <= (180 + tolerance)) && (controllerAngle >= (180 - tolerance))){
            controllerAngle = 180;
        }
        else if((controllerAngle <= (270 + tolerance)) && (controllerAngle >= (270 - tolerance))){
            controllerAngle = 270;
        }
        return controllerAngle;
    }
    //******************************************************
    public double getControllerMagnitudeDeadBand(double yStick, double xStick, double tolerance) {
        double controllerMagnitude = 0;

        double controllerAngle = getControllerAngleDeadband(yStick, xStick, tolerance);
        if((controllerAngle == 0) || (controllerAngle == 180)) {
            controllerMagnitude = Math.abs(yStick);
        }
        else if((controllerAngle == 90) || (controllerAngle == 270)) {
            controllerMagnitude = Math.abs(xStick);
        }
        else {
            // get our Pythagorean on
            controllerMagnitude = Math.sqrt(Math.pow(yStick, 2) + Math.pow(xStick, 2));
            // Normalize the number to keep it 1 or smaller by dividing by max, Max = Sqrt(1^2 + 1^2)
            controllerMagnitude = controllerMagnitude / (1.4142145624);
            // Decrease sensitivity at low magnitude
            if(controllerMagnitude < 0.4)
            {
                double sensitivityFactor = 2;
                controllerMagnitude = controllerMagnitude/sensitivityFactor;
            }
        }
        return controllerMagnitude;
    }


    //******************************************************
    public double getControllerMagnitude(double yStick, double xStick) {
        double controllerMagnitude = 0;
        if (yStick == 0) {
            controllerMagnitude = Math.abs(xStick);
        } else if (xStick == 0) {
            controllerMagnitude = Math.abs(yStick);
        } else {
            // get our Pythagorean on
            controllerMagnitude = Math.sqrt(Math.pow(yStick, 2) + Math.pow(xStick, 2));
            // Normalize the number to keep it 1 or smaller by dividing by max, Max = Sqrt(1^2 + 1^2)
            controllerMagnitude = controllerMagnitude / (1.4142145624);
            // Decrease sensitivity at low magnitude
            if(controllerMagnitude < 0.4)
            {
                double sensitivityFactor = 2;
                controllerMagnitude = controllerMagnitude/sensitivityFactor;
            }
        }
        return controllerMagnitude;
    }
//****************************************************
    public void launchParticle() {
        int stepCount = 100; //KT: was 100
        particleShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //KT:************try?

        // First we fire
        while(launcherPositionSensor.getValue() != 0){
            particleShooter.setPower(.7);//KT: Was 1.0
            waitForTick(10);
        }
        particleShooter.setPower(0);
 //       waitForTick(20);
        // debounce
        int count = 0;
        while(count < 10) {
            waitForTick(10);
            if (launcherPositionSensor.getValue() == 0) {
                count++;
            } else {
                count = 0;
            }
        }


        // now we reset
        // turn the motor until the switch disengages
        while(launcherPositionSensor.getValue() == 0){
            particleShooter.setPower(1); //KT: was 1.0
            waitForTick(10);
        }



    }
// ***************************************
    public void mecanumDrive(double direction, double speed, double rotation)//, double heading, boolean enableHeadingCorrection)
    {

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // front left and rear right are always the same speed but inverted because the
        // motors are reversed.  front right and rear left are the same
        // Reduce the rotation so it is'nt instantaneous
        double frontLeft = speed * Math.sin(Math.toRadians(direction) + (Math.PI / 4));// + (rotation * 0.50);
        double frontRight = speed * Math.cos(Math.toRadians(direction) + (Math.PI / 4));// - (rotation * 0.50);

        // invert for the complimentary wheels
        double rearRight = frontLeft * -1;
        double rearLeft = frontRight * -1;

//        double frontLeft = rearRight * -1;
//        double frontRight = rearLeft * -1;


        // add to front and rear left
        // subtract from front and rear right
        // Max it out at 50% of full turn
        frontLeft += rotation * .50;
        rearLeft -= rotation * .50;
        rearRight += rotation * .50;
        frontRight -= rotation * .50;

        // scale the values
        double maximum = Math.max(Math.max(Math.abs(frontLeft),Math.abs(frontRight)), Math.max(Math.abs(rearLeft),Math.abs(rearRight)));
        if(maximum > 1) { // was maximum != 0 but this will always result in full speed
            frontLeft = frontLeft / maximum;
            frontRight = frontRight / maximum;
            rearLeft = rearLeft / maximum;
            rearRight = rearRight / maximum;
        }

        // to make the robot rotate while moving add or subtract
        // a scalar value to the right pair and left pair of wheels
        frontRightMotor.setPower(frontRight);
        frontLeftMotor.setPower(frontLeft);
        rearRightMotor.setPower(rearRight);
        rearLeftMotor.setPower(rearLeft);
    }

public void autoStrafeRight(int numberOfSteps, double speed)// KT: Made lots of changes!
{
    // Set the motor directions //KT: Changed  moved to being first??
    frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForTick(40);//KT: ADDED to try
    // define the step counts
    int frontRightMotorStepsToDo = frontRightMotor.getCurrentPosition() + numberOfSteps; //KT:was-
    int frontLeftMotorStepsToDo = frontLeftMotor.getCurrentPosition() + numberOfSteps;
    int rearRightMotorStepsToDo = rearRightMotor.getCurrentPosition() + numberOfSteps;
    int rearLeftMotorStepsToDo = rearLeftMotor.getCurrentPosition() + numberOfSteps; //KT:was-
    waitForTick(40);
    // set the target steps
    frontRightMotor.setTargetPosition(frontRightMotorStepsToDo);
    frontLeftMotor.setTargetPosition(frontLeftMotorStepsToDo);
    rearRightMotor.setTargetPosition(rearRightMotorStepsToDo);
    rearLeftMotor.setTargetPosition(rearLeftMotorStepsToDo);
    waitForTick(40);
    // Turn On RUN_TO_POSITION
//    setDriveMotorsRunToPosition();
    frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    waitForTick(40);

    // reset the timeout time and start motion.
    setDriveMotorsToCommonSpeed(Math.abs(speed));

    // keep looping while we are still active, and there is time left, and both motors are running.
    while (frontRightMotor.isBusy() && frontLeftMotor.isBusy() && rearRightMotor.isBusy() && rearLeftMotor.isBusy()) {
        waitForTick(5);   // sit and wait for them to finish
    }

    // Stop all motion;
    setDriveMotorsToCommonSpeed(0);
    waitForTick(40);
    // Turn off RUN_TO_POSITION
    setDriveMotorsUsingEncoders();
   moveForward(0);  // KT: thought I would try to see if it matters
}
    public void autoStrafeLeft(int numberOfSteps, double speed)// KT: Made lots of changes!
    {
        // Set the motor directions //KT: Changed  moved to being first??
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForTick(40);//KT: ADDED to try
        // define the step counts
        int frontRightMotorStepsToDo = frontRightMotor.getCurrentPosition() + numberOfSteps; //KT:was-
        int frontLeftMotorStepsToDo = frontLeftMotor.getCurrentPosition() + numberOfSteps;
        int rearRightMotorStepsToDo = rearRightMotor.getCurrentPosition() + numberOfSteps;
        int rearLeftMotorStepsToDo = rearLeftMotor.getCurrentPosition() + numberOfSteps; //KT:was-
        waitForTick(40);
        // set the target steps
        frontRightMotor.setTargetPosition(frontRightMotorStepsToDo);
        frontLeftMotor.setTargetPosition(frontLeftMotorStepsToDo);
        rearRightMotor.setTargetPosition(rearRightMotorStepsToDo);
        rearLeftMotor.setTargetPosition(rearLeftMotorStepsToDo);
        waitForTick(40);
        // Turn On RUN_TO_POSITION
//    setDriveMotorsRunToPosition();
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForTick(40);

        // reset the timeout time and start motion.
        setDriveMotorsToCommonSpeed(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (frontRightMotor.isBusy() && frontLeftMotor.isBusy() && rearRightMotor.isBusy() && rearLeftMotor.isBusy()) {
            waitForTick(5);   // sit and wait for them to finish
        }

        // Stop all motion;
        setDriveMotorsToCommonSpeed(0);
        waitForTick(40);
        // Turn off RUN_TO_POSITION
        setDriveMotorsUsingEncoders();
        moveForward(0);  // KT: thought I would try to see if it matters
    }


}


