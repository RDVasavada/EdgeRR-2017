package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Rotation test")
//@Disabled
public class RotationTest extends LinearOpMode {

    // The hardware object
    EdgeBot2 robot;

    // Timer
    ElapsedTime timer;

    @Override
    public void runOpMode() {
        // Initialize the hardware object
        robot = new EdgeBot2();
        robot.init(hardwareMap, this);

        timer = new ElapsedTime();

        waitForStart();

        robot.rotateToGyroHeading2(90, 1, telemetry);
        robot.rotateToGyroHeading2(0, 1, telemetry);
        robot.rotateToGyroHeading2(180, 1, telemetry);
    }
}