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

@Autonomous(name = "Extra blocks test")
//@Disabled
public class ExtraBlocksTest extends LinearOpMode {

    // The hardware object
    EdgeBot robot;

    // Timer
    ElapsedTime period;

    boolean blockGrabbed;

    @Override
    public void runOpMode() {
        // Initialize the hardware object
        robot = new EdgeBot();
        robot.init(hardwareMap, this);

        // Wait for the start button to be pressed.
        waitForStart();

        blockGrabbed = false;

        // Open the clamp servos halfway
        robot.openClampServosHalfway();

        // Turn around
        robot.rotateClockwiseEncoder(180, 0.5, telemetry);

        // Start the timer
        period = new ElapsedTime();
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

        robot.driveForwardForInches(3, 0.5);

        robot.closeClampServos();

        robot.driveBackwardForInches(5, 0.4);

        robot.rotateToGyroHeading(0, 0.5, telemetry);

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

        robot.openClampServos();

        robot.driveBackwardForInches(3, 0.3);

        //robot.driveBackwardForInches(3, 0.3);
        //robot.rotateClockwiseEncoder(90, 0.3, telemetry);
        //robot.rotateClockwiseEncoder(90, 0.3, telemetry);
    }
}

