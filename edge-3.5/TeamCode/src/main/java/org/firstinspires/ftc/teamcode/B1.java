package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "B-1", group = "Sensor")
//@Disabled
public class B1 extends LinearOpMode {

    // The hardware object
    EdgeBot robot;

    @Override
    public void runOpMode() {
        // Initialize the hardware object
        robot = new EdgeBot();
        robot.init(hardwareMap);

        // Wait for the start button to be pressed.
        waitForStart();

        robot.waitForTick(10);

        robot.openClampServos(1);

        robot.waitForTick(1000);

        robot.driveForwardForSteps(1500, 0.5);

        robot.waitForTick(100);

        robot.rotateClockwiseEncoder(90, 0.3, telemetry);
        robot.driveForwardForSteps(300, 0.2);
        robot.closeClampServos();
    }
}