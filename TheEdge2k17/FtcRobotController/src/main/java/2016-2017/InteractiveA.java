import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Interactive Mode A", group="Linear Opmode")
@Disabled
public class InteractiveA extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    theEdge robot = new theEdge();

    @Override
    public void runOpMode() throws InterruptedException {
        double xDirectionLeftStick;
        double yDirectionLeftStick;
        double xDirectionRightStick;
        double particleLiftSpeed;
        double particleShooterSpeed;
        double bigBallLift;

        // Initialize the hardware map, Gyro, and motors
        robot.init(hardwareMap);
        robot.calibrateGyro();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Ready waiting for start...");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Track system motion and heading states for gyro compensation
        double originalHeading = 0;
        boolean isMovingForwardOrReverse = false;

        // Servo stuff
        final double MAX_POWER = 1;     // Maximum rotational position
        final double MIN_POWER = -1;     // Minimum rotational position
        double servoPower = 0;
        boolean servoDirection = true; // true = forward

        // Do something with the servo
        robot.ballPull.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.ballPull.setPower(0);

        boolean loadNextParticleLastState = false;
        boolean beaconHitterRLastState = false;
        boolean beaconHitterLLastState = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // **** Particle Lift ****************************************************
            particleLiftSpeed = gamepad1.left_trigger;
            robot.particleLift.setPower(particleLiftSpeed);
            robot.particleShift.setPower(particleLiftSpeed); //added for new particle shift

            particleShooterSpeed = gamepad1.right_trigger;
            robot.particleShooter.setPower(particleShooterSpeed);

            bigBallLift = -gamepad2.right_trigger + gamepad2.left_trigger;
            robot.ballLift.setPower(bigBallLift);

            if (gamepad1.a) {
                beaconHitterRLastState = true;
                robot.setBeaconHitterR(beaconHitterRLastState);
            }
            else {
                beaconHitterRLastState = false;
                robot.setBeaconHitterR(beaconHitterRLastState);
            }
            if (gamepad1.b) {
                beaconHitterLLastState = true;
                robot.setBeaconHitterL(beaconHitterLLastState);
            }
            else {
                beaconHitterLLastState = false;
                robot.setBeaconHitterL(beaconHitterLLastState);
            }
            if (gamepad2.a) {
                loadNextParticleLastState = true;
                robot.loadNextParticle(loadNextParticleLastState);
            }
            else {
                loadNextParticleLastState = false;
                robot.loadNextParticle(loadNextParticleLastState);
            }

            if (gamepad2.x) {
                // This should rotate the motor one full rev and stop - Neverest 40 (280*4= 1120) for 1 Rev.  Then 4:1 = 4480 for full rev of output shaft
                robot.particleShootByEncoder(2240); // value for half of a rev. since we have two hits per rev
               // loadNextParticleLastState = true;
                //robot.loadNextParticle(loadNextParticleLastState);
                //sleep(200);
                //loadNextParticleLastState = true;
                //robot.loadNextParticle(loadNextParticleLastState);
                // KT: Commented out all of this because this handled in particleshooterbyencoder

            }



           // else {
            //    loadNextParticleLastState = false;
              //  robot.loadNextParticle(loadNextParticleLastState);
           // }
            if(gamepad2.b) {
              //  robot.launchParticle(); // KT: Need to fix switch problem
                robot.particleShootByEncoder1(200);// Will Allow advancing of arm

            }

            // Increment the position of the servo
            if (gamepad2.dpad_up) {
                servoPower = MAX_POWER;
                robot.ballPull.setPower(servoPower);
            }
            else if (gamepad2.dpad_down) {
                servoPower = MIN_POWER;
                robot.ballPull.setPower(servoPower);
            }
            else if (!gamepad2.dpad_up && !gamepad2.dpad_down)  // This is going to cause a problem, you cant put anything below this
            {
                servoPower = 0;
                robot.ballPull.setPower(servoPower);
            }

            // Get the current heading from the Gyro
            double currentHeading = robot.getCurrentHeading();

            // Read the user input
            xDirectionLeftStick = gamepad1.left_stick_x;
            yDirectionLeftStick = gamepad1.left_stick_y;
            xDirectionRightStick = gamepad1.right_stick_x;

            // Get the angle and magnitude from the controller
            double leftControllerAngle = robot.getControllerAngleDeadband(yDirectionLeftStick, xDirectionLeftStick,45);
            double leftControllerMagnitude = robot.getControllerMagnitudeDeadBand(yDirectionLeftStick, xDirectionLeftStick,45);

            // Set the target heading according to the controller
            double targetHeading = leftControllerAngle;

            // TODO: Have this look at the right stick as well and then get rid of the else if for xDirection
            if (leftControllerAngle != -1) {
                robot.mecanumDrive(targetHeading, leftControllerMagnitude, xDirectionRightStick);
            }
            else if (xDirectionRightStick > 0) {
                robot.rotateClockwise(xDirectionRightStick);
                isMovingForwardOrReverse = false;   // make sure we know that the robot has stopped
            }
            else if (xDirectionRightStick < 0) {
                robot.rotateCounterClockwise(Math.abs(xDirectionRightStick));
                isMovingForwardOrReverse = false;   // make sure we know that the robot has stopped
            }
            else // they are perfectly equal or 0
            {
                robot.allWheelsStop();
                isMovingForwardOrReverse = false;   // make sure we know that the robot has stopped
            }

            // Refresh the data displayed on the driver station
            telemetry.addData("Heading", "%.2f", currentHeading);
            telemetry.addData("Servo P", "%.2f", servoPower);
//            telemetry.addData("Target", "%.2f", targetHeading);
//            telemetry.addData("Error", "%d", headingError);
            telemetry.addData("rotation", "%.2f", xDirectionRightStick);
            telemetry.addData("x", "%.2f", xDirectionLeftStick);
            telemetry.addData("y", "%.2f", yDirectionLeftStick);
            telemetry.addData("Angle: ", "%.2f", leftControllerAngle);
            telemetry.addData("Left Mag: ", "%.2f", leftControllerMagnitude);
//            telemetry.addData("Right Mag: ", "%.2f", rightControllerMagnitude);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
            idle();
        }
    }
}
