package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "A-2")
//@Disabled
public class A2 extends LinearOpMode {

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

        robot.driveBackwardForSteps(925, 0.5);

        robot.waitForTick(100);

        robot.rotateClockwiseEncoder(90, 0.3, telemetry);
        robot.driveForwardForSteps(200, 0.2);

        robot.waitForTick(50);

        robot.openClampServos(1);

        robot.waitForTick(500);

        robot.driveBackwardForSteps(100, 0.3);
    }
}