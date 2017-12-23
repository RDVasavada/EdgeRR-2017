package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autonomous Jewel Test", group = "Sensor")
//@Disabled
public class JewelTestAuton extends LinearOpMode {

    // The hardware object
    EdgeBot robot;

    // Whether or not the ball has been flipped yet
    boolean jewelFlipped;

    // Whether the ball orientation has yet been determined
    boolean orientationDetermined;

    // The location of the red ball
    boolean redOnLeft;

    // Timer
    ElapsedTime period;

    @Override
    public void runOpMode() {
        // Initialize the hardware object
        robot = new EdgeBot();
        robot.init(hardwareMap);

        jewelFlipped = false;
        orientationDetermined = false;

        // Wait for the start button to be pressed.
        waitForStart();

        // Turn on the LEDs
        //robot.turnOnLEDs();

        // Lower the lift servo
        robot.lowerJewelArm();

        // Start the timer
        period = new ElapsedTime();
        period.reset();

        // Loop and read the RGB data.
        while (opModeIsActive() && !jewelFlipped) {

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

                if (period.seconds() < 10) {
                    // Check if both sensors have determined a color
                    if (leftSensorRed && rightSensorBlue) {
                        redOnLeft = true;
                        orientationDetermined = true;
                    } else if (leftSensorBlue && rightSensorRed) {
                        redOnLeft = false;
                        orientationDetermined = true;
                    }

                    telemetry.addData("Verifying ", "both colors");
                } else {
                    // Check if one sensor has determined a color
                    if ((leftSensorRed && !rightSensorRed) || (!leftSensorBlue && rightSensorBlue)) {
                        redOnLeft = true;
                        orientationDetermined = true;
                    } else if ((!leftSensorRed && rightSensorRed) || (leftSensorBlue && !rightSensorBlue)) {
                        redOnLeft = false;
                        orientationDetermined = true;
                    }
                }

                telemetry.update();

            } else { // Color identified

                robot.displayColorValues(telemetry);

                robot.waitForTick(100);

                // Act depending on the orientation of the balls
                if (redOnLeft) {
                    robot.jewelFlipRight();
                    telemetry.addLine()
                            .addData("Red detected on ", "left")
                            .addData("Blue detected on ", "right");
                } else {
                    robot.jewelFlipLeft();
                    telemetry.addLine()
                            .addData("Blue detected on ", "left")
                            .addData("Red detected on ", "right");
                }

                telemetry.update();

                // Turn off the LEDs
                //robot.turnOffLEDs();

                // Keep the arm in place for two seconds
                robot.waitForTick(1000);

                // Return the servos to their original position
                robot.resetJewelServos();

                jewelFlipped = true;
            }
        }

    }
}