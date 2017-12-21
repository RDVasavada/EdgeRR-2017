import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Straight and Park Delay 15", group="Autonomous")  // @Autonomous(...) is the other common choice
@Disabled
public class AutoStraightNParksDelay15 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    theEdge robot = new theEdge();

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

// particlShootByEncoder
            telemetry.addData("Straight ", "Park");
            telemetry.update();

          // Move forward then shoot - one rev = 2240 / (4* 3.14) = 178 pulses per inch
           sleep(15000);
            robot.autoDriveForward(4000,1);

            // This should rotate the motor one full rev and stop - Neverest 40 (280*4= 1120) for 1 Rev.  Then 4:1 = 4480 for full rev of output shaft
            robot.particleShootByEncoder(2240); // value for half of a rev. since we have two hits per rev

            sleep(200);
            robot.particleShootByEncoder(2240); //Can shorten for 2nd shot to speed loop up

            // Move forward and bump robot
            robot.autoDriveForward(600,1);
            sleep(500);
           // robot.rotateToDirection(telemetry, 30);

            // Park on Ramp
            robot.autoDriveForward(1000,1);
     //     Stop
            robot.allWheelsStop();
         stop();
            //**************************************************************************************

//stop();
idle();
        }

    }

}