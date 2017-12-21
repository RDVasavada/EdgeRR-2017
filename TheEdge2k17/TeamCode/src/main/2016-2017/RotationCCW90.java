import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// This program will shoot twice angle to wall then turn 90D, advance till 30cm from wall, strafe right to line sensor

@Autonomous(name="Rotate CCW90 Test", group="Autonomous")  // @Autonomous(...) is the other common choice
@Disabled
public class RotationCCW90 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    theEdge robot = new theEdge();
    int check = 1;
    int check2 = 1;
    int exit = 1;

    @Override
    public void runOpMode() {
        // Initialize the hardware map, Gyro, and motors
        // Send a message to the controller...
        robot.init(hardwareMap);
        double preCalibratedHeading = robot.gyro.getHeading();
        telemetry.addData("Pre-Cal Heading: ", "%.2f", preCalibratedHeading);
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        robot.calibrateGyro();
        double postCalibratedHeading = robot.gyro.getHeading();
        telemetry.addData("Pre-Cal Heading:  ", "%.2f", preCalibratedHeading);
        telemetry.addData("Post-Cal Heading: ", "%.2f", postCalibratedHeading);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (Eddie (driver) presses PLAY)
        waitForStart();
        // ***************************************************************************************************
        // This loop will do all the magic stuff!
        while (opModeIsActive()) {
            // robot.loadNextParticle(true);
            double realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);
            double blueLowRange = 200;
            double blueHighRange = 255; //was 230
            double redLow1Range = 290;
            double redHigh1Range = 370;
            double redLow2Range = .01;
            double redHigh2Range = 12; //was 10
            double beaconDistStart = 7.5; //This is the distance to move forward to ensure you've pushed the button was 7.4
            double distAve = 0;
            long hitDelay = 500; // delay to allow servos to flip on beacons
//

//************************************************************************************************************
//****  STEP 1 ************ Drive straight, shoot, rotate to line
            telemetry.addData("Test 90 CCW ", " ");
            telemetry.update();


//**** STEP 2  ************Rotate and drive toward wall

/*
            telemetry.addData("Heading Start  ", "%d", robot.gyro.getHeading());
            telemetry.update();

            robot.rotateClockwiseEncoder(50, 1);  //was 55

            robot.waitForTick(150);
            telemetry.addData("Heading After  ", "%d", robot.gyro.getHeading());
            telemetry.update();
            //    sleep(5000);
            //    robot.allWheelsStop();
            robot.rotateByEncoderCorrection(52, 0.5, 2); // correct to get closer to 52
            telemetry.addData("Heading Adj  ", "%d", robot.gyro.getHeading());
            telemetry.update();
            //  robot.allWheelsStop();
            // robot.waitForTick(1000);
            //   sleep(5000);


            //   sleep(5000);  //    sleep(5000);
            telemetry.addData("Heading BEFORE:  ", "%d", robot.gyro.getHeading());
            telemetry.update();

            robot.rotateClockwiseEncoder(32, 1); // rotate 90 - 52= 38
            //   robot.allWheelsStop();

            robot.waitForTick(150);
            telemetry.addData("Heading after:  ", "%d", robot.gyro.getHeading());
            telemetry.update();

            //    sleep(5000);
            robot.rotateByEncoderCorrection(90, 0.5, 2); // correct to get closer to 90
            telemetry.addData("Heading ADJ:  ", "%d", robot.gyro.getHeading());
            telemetry.update();
            sleep(5000);
*/
         //   robot.rotateCounterClockwise(3000);
         //   sleep(2000);

//***************************** nOW ccw ******************************************
            telemetry.addData("Heading to 90 Start  ", "%d", robot.gyro.getHeading());
            telemetry.update();
            // robot.rotateToDirection(telemetry, 57, 30, .7, .05);// Try being agressive
        //    robot.rotateCounterClockwise(.9);
         //   sleep(2000);
          //  robot.allWheelsStop();
            sleep(2000);
            //  robot.autoDriveForward(10, 1);
            robot.rotateClockwiseEncoder(90, .9);  //was 55
            //  telemetry.addData("Range Sensor ", "%.2f", robot.rangeSensor.getDistance(DistanceUnit.CM));
            sleep(2000);
            robot.rotateByEncoderCorrection(90, 0.2, 2); // correct to get closer to 52
            robot.waitForTick(150);
            sleep(2000);
            telemetry.addData("Heading after  90CW After  ", "%d", robot.gyro.getHeading());
            telemetry.update();

            sleep(2000);
          //  robot.autoDriveForward(10, 1);
            robot.rotateClockwiseEncoder(80, .9);  //was 55
            //  telemetry.addData("Range Sensor ", "%.2f", robot.rangeSensor.getDistance(DistanceUnit.CM));
            sleep(2000);
            robot.rotateByEncoderCorrection(180, 0.2, 2); // correct to get closer to 52
            robot.waitForTick(150);
            sleep(2000);
            telemetry.addData("Heading after  90CW After  ", "%d", robot.gyro.getHeading());
            telemetry.update();

          //  robot.autoDriveForward(10, 1);
            robot.rotateCounterClockwiseEncoder(80, .9);  //was 55
            //  telemetry.addData("Range Sensor ", "%.2f", robot.rangeSensor.getDistance(DistanceUnit.CM));
            sleep(2000);
            robot.rotateByEncoderCorrection(90, 0.2, 2); // correct to get closer to 52
            robot.waitForTick(150);
            sleep(2000);
            telemetry.addData("Heading after  90CW After  ", "%d", robot.gyro.getHeading());
            telemetry.update();
            telemetry.addData("Heading to 90 Start  ", "%d", robot.gyro.getHeading());
            telemetry.update();
            // robot.rotateToDirection(telemetry, 57, 30, .7, .05);// Try being agressive
            //    robot.rotateCounterClockwise(.9);
            //   sleep(2000);
            //  robot.allWheelsStop();

            sleep(2000);
            //  robot.autoDriveForward(10, 1);
            robot.rotateClockwiseEncoder(100, .9);  //was 55
            //  telemetry.addData("Range Sensor ", "%.2f", robot.rangeSensor.getDistance(DistanceUnit.CM));
            sleep(2000);
            robot.rotateByEncoderCorrection(180, 0.2, 2); // correct to get closer to 52
            robot.waitForTick(150);
            sleep(2000);
            telemetry.addData("Heading after  90CW After  ", "%d", robot.gyro.getHeading());
            telemetry.update();

            //  robot.autoDriveForward(10, 1);
            robot.rotateCounterClockwiseEncoder(100, .9);  //was 55
            //  telemetry.addData("Range Sensor ", "%.2f", robot.rangeSensor.getDistance(DistanceUnit.CM));
            sleep(2000);
            robot.rotateByEncoderCorrection(90, 0.2, 2); // correct to get closer to 52
            robot.waitForTick(150);
            sleep(2000);
            telemetry.addData("Heading after  90CW After  ", "%d", robot.gyro.getHeading());
            telemetry.update();

            //robot.rotateCounterClockwise(.9);
            //sleep(2000);
            //robot.allWheelsStop();
            //    robot.allWheelsStop();
        // robot.rotateByEncoderCorrection(90, 0.5, 2); // correct to get closer to 52
          //  telemetry.addData("Heading CCW Adj  ", "%d", robot.gyro.getHeading());
            //telemetry.update();

        //  sleep(5000);
       /*     telemetry.addData("Heading to to 90 CCW  ", "%d", robot.gyro.getHeading());
            telemetry.update();
            robot.rotateCounterClockwiseEncoder(90, 1);  //was 55
            sleep(200);
            telemetry.addData("Should be back to 0  ", "%d", robot.gyro.getHeading());
            telemetry.update();
/*
            telemetry.addData("Heading BEFORE:  ", "%d", robot.gyro.getHeading());
            telemetry.update();

            robot.rotateCounterClockwiseEncoder(90, 1); // rotate 90 - 52= 38

            robot.waitForTick(150);
            telemetry.addData("Heading after:  ", "%d", robot.gyro.getHeading());
            telemetry.update();

            //    sleep(5000);
           robot.rotateByEncoderCorrection(0, 0.5, 2); // correct to get closer to 90
            telemetry.addData("Heading ADJ:  ", "%d", robot.gyro.getHeading());
            telemetry.update();

//*******************************************************************
//  robot.allWheelsStop();
            sleep(5000); //
            telemetry.addData("Heading ADJ2:  ", "%d", robot.gyro.getHeading());
            telemetry.update();
            sleep(5000); //
            double sensor1 = robot.getRGB1HSV(telemetry);
            robot.waitForTick(15);
            double sensor2 = robot.getRGB2HSV(telemetry);
            robot.waitForTick(15);

            telemetry.addData("Sensor 1", "%.2f", sensor1);
            telemetry.addData("Sensor 2", "%.2f", sensor2);
*/
            telemetry.update();
           // sleep(10000);
            stop();


        }

    }
}