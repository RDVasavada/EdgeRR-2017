package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "A-2", group = "Sensor")
//@Disabled
public class A2 extends LinearOpMode {

    // The hardware object
    EdgeBot robot;

    @Override
    public void runOpMode() {
        // Initialize the hardware object
        robot = new EdgeBot();
        robot.init(hardwareMap);

        int x = 1;

        // Wait for the start button to be pressed.
        waitForStart();

        robot.waitForTick(10);

        robot.closeClampServos();

        robot.waitForTick(1000);

        if (x == 0) {
            robot.driveForwardForSteps(1200, 0.5);

            robot.waitForTick(100);

            robot.rotateClockwiseEncoder(102, 0.3, telemetry);
            robot.driveForwardForSteps(300, 0.2);
            robot.openClampServos(1);
        } else if (x == 1) {
            robot.driveForwardForSteps(1800, 0.5);

            robot.waitForTick(100);

            robot.rotateClockwiseEncoder(90, 0.3, telemetry);
            robot.driveForwardForSteps(300, 0.2);
            robot.openClampServos(1);
        }
    }
}