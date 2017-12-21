import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// This program will shoot twice angle to wall then turn 90D, advance till 30cm from wall, strafe right to line sensor

@Autonomous(name="Red Two 2 New2", group="Autonomous")  // @Autonomous(...) is the other common choice
@Disabled
public class RedTwo2NEW2 extends LinearOpMode {

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
        telemetry.addData("Pre-Cal Heading: ","%.2f", preCalibratedHeading);
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        robot.calibrateGyro();
        double postCalibratedHeading = robot.gyro.getHeading();
        telemetry.addData("Pre-Cal Heading:  ","%.2f", preCalibratedHeading);
        telemetry.addData("Post-Cal Heading: ","%.2f", postCalibratedHeading);
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
            double beaconDistStart = 7.2; //This is the distance to move forward to ensure you've pushed the button was 7.4
            double distAve = 0;
            long hitDelay = 500; // delay to allow servos to flip on beacons
/*

Worries :
- Timeouts if something goes wrong
- Detecting a big ball if in the way of the distance measurement
-

 */

//************************************************************************************************************
//****  STEP 1 ************ Drive straight, shoot, rotate to line
            telemetry.addData("Straight ", "shoot 2");
            telemetry.update();

            // Move forward then shoot - one rev = 2240 / (4* 3.14) = 178 pulses per inch
            robot.autoDriveForward(2100, 1); //was 2300 JUST ADJUSTED LAUNCH ANGLE
            // Shoot 2 particles
            // This should rotate the motor one full rev and stop - Neverest 40 (280*4= 1120) for 1 Rev.  Then 4:1 = 4480 for full rev of output shaft
           robot.particleShootByEncoder(2240); // value for half of a rev. since we have two hits per rev

           robot.particleShootByEncoder(800); //Was 2240 Can shorten for 2nd shot to speed loop up
//**** STEP 2  ************Rotate and drive toward wall
        // sleep(3000);
            telemetry.addData("Rotate ", "Drive to 20cm from wall");
            telemetry.update();

           // robot.rotateToDirection(telemetry, 57, 30, .7, .05);// Try being agressive
           robot.rotateCounterClockwiseEncoder(55,1);  //was 55
            telemetry.addData("Range Sensor ", "%.2f", robot.rangeSensor.getDistance(DistanceUnit.CM));
            robot.waitForTick(150);
            telemetry.addData("Heading After  ", "%d", robot.gyro.getHeading());
            telemetry.update();
          // sleep(3000);

  //***************** First Turn          *****************
            //  robot.rotateByEncoderCorrection(51,0.5,2); //correct to get closer to 52
            robot.rotateByEncoderCorrection(310,0.5,2);
            telemetry.addData("Heading Adj  ", "%d", robot.gyro.getHeading());
            telemetry.update();
         //   robot.allWheelsStop();
           // robot.waitForTick(1000);
        // sleep(5000);

            // sleep(5000);
            // sleep(1000);
            robot.autoDriveForward(2350, 1);  // Check to see how close to wall was 2250

            // Rotate back to 90
           robot.allWheelsStop();
            robot.waitForTick(50);
 //************** Second Turn *************************************
            //   sleep(5000);  //    sleep(5000);
            telemetry.addData("Heading BEFORE:  ", "%d", robot.gyro.getHeading());
            telemetry.update();
           robot.rotateCounterClockwiseEncoder(38,1); // rotate 90 - 52= 38...was 32
       //     robot.allWheelsStop();
           //robot.waitForTick(150);
            telemetry.addData("Heading after:  ", "%d", robot.gyro.getHeading());
            telemetry.update();
            robot.waitForTick(150);
          //  sleep(3000);
        //robot.rotateByEncoderCorrection(270,0.5,2); // correct to get closer to -90
         //   robot.rotateToDirection(telemetry, 270, 30, .2, .1);
            telemetry.addData("Heading ADJ:  ", "%d", robot.gyro.getHeading());
            telemetry.update();

            robot.allWheelsStop();
//******************************************************************************
          //  sleep(5000); //

           //  robot.rotateToDirectionCloseEnough(telemetry, 90,2);
           // robot.rotateToDirection(telemetry, 90, 30, .2, .1)); // Turn towards wall
            robot.waitForTick(20);
            robot.allWheelsStop();
            telemetry.addData("Range Sensor ", "%.2f", robot.rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Heading:  ", "%d", robot.gyro.getHeading());
            telemetry.update();
            //     sleep(5000);

            // Distance from wall *********************************************************

//            telemetry.addData("Range sensor... ", "??");
//            telemetry.update();
//            robot.rangeSensor(telemetry);
//            telemetry.update();
//            sleep(10); //was 100
            realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);
            // robot.moveForward(.2);

            while ((realDist > 24) && opModeIsActive()) {
                realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);
                robot.moveForward(.5);
                robot.waitForTick(10);
            }
            while ((realDist < 14) && opModeIsActive()) {
                realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);
                robot.moveBackward(.5);
                robot.waitForTick(10);
            }
            telemetry.addData("Range Sensor ", "%.2f", robot.rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Heading:  ", "%d", robot.gyro.getHeading());
            telemetry.update();

            robot.allWheelsStop();
            robot.waitForTick(10);

// ********************** STEP 3  Find White Line*****************************
            telemetry.addData("Line Tracker... ", "??");
            telemetry.update();
            robot.strafeRight(.3); // CAUTION WILL GO FOREVER
            // Need to make a timeout
            while ((!robot.lineTrack(telemetry)) && opModeIsActive()) {
                telemetry.addData("Range Sensor ", "%.2f", robot.rangeSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Heading:  ", "%d", robot.gyro.getHeading());
                telemetry.update();
                robot.waitForTick(20);
            }
//*************************Correct distance  *************************************************
            robot.allWheelsStop();
// Move to 20cm from wall
            telemetry.addData("Range Sensor ", "%.2f", robot.rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Heading:  ", "%d", robot.gyro.getHeading());
            telemetry.update();
            //sleep(20);
            realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);

            while ((realDist > 20) && opModeIsActive()) {
                realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);
                robot.moveForward(.5);
                robot.waitForTick(20);
            }
            while ((realDist < 12) && opModeIsActive())//was 17
            {
                realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);
                robot.moveBackward(.5);
                robot.waitForTick(20);
            }
//*******************************************************
            robot.allWheelsStop();
            robot.waitForTick(20);

            //***************************************************************************


// **** STEP 4    Sense Colors and go and push 1st Beacon!!  *****************************
            //robot.waitForTick(5000);
            //robot.waitForTick(5000);
            //while ((check2 == 1) && opModeIsActive()) {
                telemetry.addLine("#1");
                telemetry.update();
                //#1
                double sensor1 = robot.getRGB1HSV(telemetry);
                robot.waitForTick(10);
                double sensor2 = robot.getRGB2HSV(telemetry);
                robot.waitForTick(10);
                telemetry.addData("Sensor 1", "%.2f", sensor1);
                telemetry.addData("Sensor 2", "%.2f", sensor2);
                telemetry.update();
            check = 1;
                while ((check == 1) && opModeIsActive())
                {
                    telemetry.addLine("#2");
                    telemetry.update();
                    //#2
                    //  Note need to decide on range of the colors at various distances
                    sensor1 = robot.getRGB1HSV(telemetry);
                    robot.waitForTick(10);
                    sensor2 = robot.getRGB2HSV(telemetry);
                    robot.waitForTick(10);
                    telemetry.addData("Sensor 1", "%.2f", sensor1);
                    telemetry.addData("Sensor 2", "%.2f", sensor2);
                    telemetry.update();
                    //sensor1 if blue and sensor2 if red, good to go
                    if (((sensor1 > blueLowRange && sensor1 < blueHighRange) && ((sensor2 > redLow1Range && sensor2 < redHigh1Range) || (sensor2 > redLow2Range && sensor2 < redHigh2Range))) && opModeIsActive()) {
                        boolean beaconHitterRLastState = false;
                        robot.setBeaconHitterR(beaconHitterRLastState);
                        boolean beaconHitterLLastState = true;
                        robot.setBeaconHitterL(beaconHitterLLastState);
                        //robot.autoDriveForward(500, .2); //Need to adjust the number for the right distance
                        robot.waitForTick(hitDelay);
                        // Try turning on particle loader
                        robot.particleShift.setPower(1);
                        robot.particleLift.setPower(1);
                        realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);
                        while ((realDist > beaconDistStart) && opModeIsActive()) {
                            realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);
                            robot.moveForward(.2);
                            robot.waitForTick(10);
                        }
                        robot.allWheelsStop();
                        // allow time for beacon servos
                        robot.autoDriveBackward(100, .4);// was 1000
                        robot.waitForTick(50);
                        robot.allWheelsStop();
                        check = 2;
                       // break;
                        //#3
                    }
                    //sensor2 if blue and sensor1 if red,good to go
                    else if (((sensor2 > blueLowRange && sensor2 < blueHighRange) && ((sensor1 > redLow1Range && sensor1 < redHigh1Range) || (sensor1 > redLow2Range && sensor1 < redHigh2Range))) && opModeIsActive()) {
                        boolean beaconHitterLLastState = false;
                        robot.setBeaconHitterL(beaconHitterLLastState);
                        boolean beaconHitterRLastState = true;
                        robot.setBeaconHitterR(beaconHitterRLastState);
                        robot.waitForTick(hitDelay);
                        // Try turning on particle loader
                        robot.particleShift.setPower(1);// allow time for beacon servos
                        robot.particleLift.setPower(1);
                        realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);
                        while (realDist > beaconDistStart && opModeIsActive()) {
                            realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);

                            robot.moveForward(.2);
                            robot.waitForTick(10);
                        }

                        robot.autoDriveBackward(100, .5);
                        robot.allWheelsStop();
                        check = 2;
                       // break;
                        //#3
                    }

                    // Try turning on particle loader
                    robot.particleShift.setPower(0);
                    robot.particleLift.setPower(0);

                }

//*******************************************************************************************************
//*******************************************************************************************************
// ****************************** Step 5 Now go to other Line beacon*********************************
            //
            robot.autoDriveBackward(800, .5);
            telemetry.addLine("#9");
            telemetry.update();
                robot.allWheelsStop();
                robot.autoStrafeRight(4000, 1); //was 4000
                robot.allWheelsStop();
// Distance from wall
                while ((realDist < 15) && opModeIsActive()) {
                    realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);
                    robot.moveBackward(.4);
                    robot.waitForTick(10);
                }
            while ((realDist > 20) && opModeIsActive()) {
                realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);
                robot.moveForward(.5);
                robot.waitForTick(20);
            }

//******************************* Look for other Line *************************************************
            robot.strafeRight(.25);//  CAUTION  will keep going forever?was .3
            telemetry.addData("Line Tracker... ", "??");
            telemetry.update();

            while (!robot.lineTrack(telemetry) && opModeIsActive()) {
                    robot.waitForTick(20);
                }
                robot.allWheelsStop();
//*******************************Rotate correction **********************************************************************
                robot.rotateToDirection(telemetry, 270, 30, .2, .1);// correct if not at Rotate back to 90
            //robot.rotateToDirectionCloseEnough(telemetry, 90,2);

            while ((realDist > 16) && opModeIsActive()) {
                    realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);
                    robot.moveForward(.3); //was .2 on 4-23-17
                    robot.waitForTick(10);
                }


                robot.allWheelsStop();
//**************************** Rotate again ************************************************
            robot.rotateToDirection(telemetry, 270, 30, .2, .1);
//****************************  Find Line AGAIN ***********************************************
            robot.strafeRight(.25);//  CAUTION  will keep going forever?was .3
            while (!robot.lineTrack(telemetry) && opModeIsActive()) {
                robot.waitForTick(20);
            }
            robot.allWheelsStop();
            telemetry.addData("Line Tracker... ", "??");
            telemetry.update();
 //*********************************************************************************************

// **** STEP 6    Sense Colors and go and push 2st Beacon!!  *****************************
//*************************************************************
                //robot.waitForTick(5000);
                check = 1;
                check2 = 1;

            while ((check == 1) && opModeIsActive())
            {
                telemetry.addLine("#2");
                telemetry.update();
                //#2
                //  Note need to decide on range of the colors at various distances
                sensor1 = robot.getRGB1HSV(telemetry);
                robot.waitForTick(10);
                sensor2 = robot.getRGB2HSV(telemetry);
                robot.waitForTick(10);
                telemetry.addData("Sensor 1", "%.2f", sensor1);
                telemetry.addData("Sensor 2", "%.2f", sensor2);
                telemetry.update();
                //sensor1 if blue and sensor2 if red, good to go
                if (((sensor1 > blueLowRange && sensor1 < blueHighRange) && ((sensor2 > redLow1Range && sensor2 < redHigh1Range) || (sensor2 > redLow2Range && sensor2 < redHigh2Range))) && opModeIsActive()) {
                    boolean beaconHitterRLastState = false;
                    robot.setBeaconHitterR(beaconHitterRLastState);
                    boolean beaconHitterLLastState = true;
                    robot.setBeaconHitterL(beaconHitterLLastState);
                    //robot.autoDriveForward(500, .2); //Need to adjust the number for the right distance
                    robot.waitForTick(hitDelay);
                    // Try turning on particle loader
                    robot.particleShift.setPower(1);
                    robot.particleLift.setPower(1);
                    realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);
                    while ((realDist > beaconDistStart) && opModeIsActive()) {
                        realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);
                        robot.moveForward(.2);
                        robot.waitForTick(20);
                    }
                    robot.allWheelsStop();
                    // allow time for beacon servos
                    robot.autoDriveBackward(100, .4);// was 1000
                    robot.waitForTick(50);
                    robot.allWheelsStop();
                    check = 2;
                    // break;
                    //#3
                }
                //sensor2 if blue and sensor1 if red,good to go
                else if (((sensor2 > blueLowRange && sensor2 < blueHighRange) && ((sensor1 > redLow1Range && sensor1 < redHigh1Range) || (sensor1 > redLow2Range && sensor1 < redHigh2Range))) && opModeIsActive()) {
                    boolean beaconHitterLLastState = false;
                    robot.setBeaconHitterL(beaconHitterLLastState);
                    boolean beaconHitterRLastState = true;
                    robot.setBeaconHitterR(beaconHitterRLastState);
                    robot.waitForTick(hitDelay); // allow time for beacon servos
                    // Try turning on particle loader
                    robot.particleShift.setPower(1);
                    robot.particleLift.setPower(1);
                    realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);
                    while (realDist > beaconDistStart && opModeIsActive()) {
                        realDist = robot.rangeSensor.getDistance(DistanceUnit.CM);

                        robot.moveForward(.2);
                        robot.waitForTick(10);
                    }

                    robot.autoDriveBackward(100, .4);
                    robot.allWheelsStop();
                    check = 2;
                    // break;
                    //#3
                }
                else if ((sensor1 > blueLowRange && sensor1 < blueHighRange) && (sensor2 > blueLowRange && sensor2 < blueHighRange))
                {
                    telemetry.addLine("in second loop");

                    robot.autoDriveBackward(100, .4);
                    //#6
                    telemetry.addLine("#6");
                    telemetry.update();
                    check = 2;
                    // break;
                }
                else if (check == 3 && opModeIsActive())
                {
                    //#8
                    telemetry.addLine("tryna break");
                    telemetry.update();
                    break;
                }
                // Rotate and drive fast to try to reach ramp




            }
            // Try turning on particle loader
            robot.particleShift.setPower(0);
            robot.particleLift.setPower(0);

            robot.autoDriveBackward(500,1);
           robot.rotateClockwiseEncoder(43,1);
            telemetry.addData("Heading After  ", "%d", robot.gyro.getHeading());
           // robot.allWheelsStop();

            //robot.rotateByEncoderCorrection(270,0.2,2);
          //  robot.allWheelsStop();
           // telemetry.addData("Heading After  ", "%d", robot.gyro.getHeading());
            telemetry.update();
            robot.autoDriveBackward(4000,1);
            robot.autoDriveBackward(1000,.4);

            robot.allWheelsStop();
              // sleep(5000);
            //**************************************************************************************


        //idle();// this was remarked out
            stop(); // this was moved up a bracket

        }



    }

}