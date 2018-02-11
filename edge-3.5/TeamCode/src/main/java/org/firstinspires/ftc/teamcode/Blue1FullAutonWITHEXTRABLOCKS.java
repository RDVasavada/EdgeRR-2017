package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "Blue 1 Full Autonomous - EXTRA BLOCKS")
//@Disabled
public class Blue1FullAutonWITHEXTRABLOCKS extends LinearOpMode {

    // The hardware object
    EdgeBot2 robot;

    // Whether or not the ball has been flipped yet
    boolean jewelFlipped;

    // Whether the ball orientation has yet been determined
    boolean orientationDetermined;

    // The location of the red ball
    boolean redOnLeft;

    // Whether a block has been grabbed
    boolean blockGrabbed;

    // Vuforia
    VuforiaLocalizer vuforiaInstance;

    // Incidental Timer
    ElapsedTime period;

    @Override
    public void runOpMode() {
        // Initialize the hardware object
        robot = new EdgeBot2();
        robot.init(hardwareMap, this);

        jewelFlipped = false;
        orientationDetermined = false;
        blockGrabbed = false;

        // Wait for the start button to be pressed.
        waitForStart();

        // Close the clamp servos
        robot.closeIntakeServos();

        // Lower the lift servo
        robot.lowerJewelArm();

        // Start the timer
        period = new ElapsedTime();
        period.reset();

        // Loop and read the RGB data.
        while (opModeIsActive() && !jewelFlipped && period.seconds() < 8) {

            if (!orientationDetermined) { // Ball orientation not determined

                // Get the hue values from each sensor
                double leftHue = robot.getLeftSensorHue();
                double rightHue = robot.getRightSensorHue();

                robot.displayColorValues(telemetry);

                // Check if left sensor is red or blue
                boolean leftSensorRed = ((leftHue > Constants.RED_LOW_1) && (leftHue < Constants.RED_HIGH_1)) || ((leftHue > Constants.RED_LOW_2) && (leftHue < Constants.RED_HIGH_2));
                boolean leftSensorBlue = (leftHue > Constants.BLUE_LOW) && (leftHue < Constants.BLUE_HIGH);

                // Check if right sensor is red or blue
                boolean rightSensorRed = ((rightHue > Constants.RED_LOW_1) && (rightHue < Constants.RED_HIGH_1)) || ((rightHue > Constants.RED_LOW_2) && (rightHue < Constants.RED_HIGH_2));
                boolean rightSensorBlue = (rightHue > Constants.BLUE_LOW) && (rightHue < Constants.BLUE_HIGH);

                // Check if one sensor has determined a color
                if ((leftSensorRed && !rightSensorRed) || (!leftSensorBlue && rightSensorBlue)) {
                    redOnLeft = true;
                    orientationDetermined = true;
                } else if ((!leftSensorRed && rightSensorRed) || (leftSensorBlue && !rightSensorBlue)) {
                    redOnLeft = false;
                    orientationDetermined = true;
                }

                telemetry.update();

            } else { // Color identified

                robot.displayColorValues(telemetry);

                robot.waitForTick(100);

                // Act depending on the orientation of the balls
                if (redOnLeft) {
                    robot.jewelFlipLeft();
                    telemetry.addLine()
                            .addData("Red detected on ", "left")
                            .addData("Blue detected on ", "right");
                } else {
                    robot.jewelFlipRight();
                    telemetry.addLine()
                            .addData("Blue detected on ", "left")
                            .addData("Red detected on ", "right");
                }

                telemetry.update();

                // Turn off the LEDs
                //robot.turnOffLEDs();

                // Keep the arm in place for one second
                robot.waitForTick(1000);

                jewelFlipped = true;
            }
        }

        // Return the servos to their original position
        robot.resetJewelServos();

        // Use vuforia to determine the block position
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AS/n2hH/////AAAAGSrRl7UksE45mS+hhFImeRYrIbqSLf6SpK5bmt3ddR+YNQPHRZ5HP4tjyNHL9ftIIosr" +
                "FELpiqEWKnPHNTNK2H6Dh+oreG4MrO0LJyuu+oZOKWxDUvBcVHKEkq6eGgK8oVLewPspuOeHCoL5/" +
                "28GHVF3vOyw/pNCDGnvZ1W/ycpR+Y7D3Onq2UAIluATXLoWjkvN3k7jEpO+bNihJ85WhgwjF6rmX/" +
                "6LBWjE4skz+bU39WQpMa1wwLj4PCKalLg/pjOmg9bjaMHCtdoaBYFMheaAkpeAKRbk9zBuHCvvaHs" +
                "kuiMmeszZe1ECsQaZJkCB39BMO9qwM5ZXrxcUUGtSwmJ+zkeVigk/mUvsNK0D8lOD";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaInstance = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = vuforiaInstance.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        boolean pictographScanned = false;

        // Set unknown as the default column
        RelicRecoveryVuMark column = RelicRecoveryVuMark.UNKNOWN;

        relicTrackables.activate();

        period.reset();

        while (opModeIsActive() && !pictographScanned && period.seconds() < 5) {
            /* This checks to see if a pictograph is visible and
            returns an enum type that can be UNKNOWN, LEFT, or RIGHT */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                // Pictograph is visible
                column = vuMark;
                pictographScanned = true;
            }

            telemetry.addData("Pictograph visible: ", vuMark);
            telemetry.update();
        }

        // Raise the lift motor
        period.reset();

        while (period.seconds() < 0.5 && opModeIsActive()) {
            robot.raiseLiftMotor();
        }

        robot.stopLiftMotor();

        // Drive backwards with center as the default column
        if (column == RelicRecoveryVuMark.LEFT) {
            robot.driveBackwardForInches(30, 0.4);
        } else if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
            robot.driveBackwardForInches(40, 0.4);
        } else if (column == RelicRecoveryVuMark.RIGHT) {
            robot.driveBackwardForInches(46, 0.4);
        }

        // Rotate clockwise 90 degrees and drive forwards into the column
        robot.rotateClockwiseEncoder(90, 0.3, telemetry);
        robot.driveForwardForInches(11, 0.2);

        // Open the clamp servos to drop the block
        robot.openIntakeServos();

        // Push the block in
        robot.driveBackwardForInches(3, 0.2);
        robot.driveForwardForInches(3, 0.2);

        // Back up
        robot.driveBackwardForInches(12, 0.3);

        // Open the clamp servos halfway
        robot.openIntakeServosHalfway();

        // Turn around
        robot.rotateClockwiseEncoder(180, 0.5, telemetry);

        // Lower the lift motor
        period.reset();
        while (period.seconds() < 0.5 && opModeIsActive()) {
            robot.lowerLiftMotor();
        }

        robot.stopLiftMotor();

        // Start the timer
        period.reset();

        while (opModeIsActive() && !blockGrabbed && period.seconds() < 5) {

            telemetry.addData("raw ultrasonic", robot.frontSensor.rawUltrasonic());
            telemetry.addData("raw optical", robot.frontSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", robot.frontSensor.cmOptical());
            telemetry.addData("inch", "%.2f inch", robot.frontSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();

            if (robot.getFrontRangeSensorDistance() > 4 && period.seconds() < 1) {
                robot.driveForwards(0.5);
            } else if (robot.getFrontRangeSensorDistance() > 4) {
                robot.driveForwards(0.3);
            } else {
                robot.stopDriveMotors();
                blockGrabbed = true;
            }

        }

        // Succ up the block
        robot.closeIntakeServos();

        // Reset the timer
        period.reset();

        while (period.seconds() < 1 && opModeIsActive()) {
            robot.intakeIn();
        }

        // Drive back and turn around
        robot.driveBackwardForInches(18, 0.4);

        robot.rotateToGyroHeading(-90, 0.5, telemetry);

        period.reset();

        while (period.seconds() < 1) {
            robot.driveForwards(0.5);
            telemetry.addData("High", "speed");
            telemetry.update();
        }

        while (period.seconds() < 5 && robot.getCryptoboxRangeSensorDistance() > 8.5) {
            robot.driveForwards(0.3);
            telemetry.addData("Low", "speed");
            telemetry.update();
        }

        robot.stopDriveMotors();

        robot.openIntakeServosHalfway();

        robot.driveBackwardForInches(3, 0.3);
    }
}