package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red 1 Block Test")
@Disabled
public class Red1BlockTest extends LinearOpMode {

    // The hardware object
    EdgeBot robot;

    // Timer
    ElapsedTime timer;

    @Override
    public void runOpMode() {
        // Initialize the hardware object
        robot = new EdgeBot();
        robot.init(hardwareMap);

        // Initialize the timer
        timer = new ElapsedTime();

        // Wait for the start button to be pressed.
        waitForStart();

        robot.waitForTick(10);

        robot.closeClampServos();

        robot.waitForTick(1000);

        timer.reset();

        while (timer.seconds() < 1) {
            robot.raiseLiftMotor(0.5);
        }

        robot.waitForTick(10);

        robot.stopLiftMotor();

        robot.waitForTick(500);

        int x = 1; // In place of the paper reading

        if (x == 1) {
            robot.driveForwardForSteps(1790, 0.5);
        } else if (x == 2) {
            robot.driveForwardForSteps(1500, 0.5);
        } else if (x == 3) {
            robot.driveForwardForSteps(1225, 0.5);
        }

        robot.waitForTick(100);

        robot.rotateClockwiseEncoder(90, 0.3, telemetry);
        robot.driveForwardForSteps(300, 0.2);

        robot.waitForTick(50);

        robot.openClampServos(1);

        robot.waitForTick(500);

        robot.driveBackwardForSteps(100, 0.3);
    }
}