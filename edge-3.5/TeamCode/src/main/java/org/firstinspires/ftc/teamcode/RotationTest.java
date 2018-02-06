package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Rotation test")
//@Disabled
public class RotationTest extends LinearOpMode {

    // The hardware object
    EdgeBot robot;

    // Timer
    ElapsedTime timer;

    @Override
    public void runOpMode() {
        // Initialize the hardware object
        robot = new EdgeBot();
        robot.init(hardwareMap, this);

        timer = new ElapsedTime();

        waitForStart();

        robot.rotateToGyroHeading(90, 0.3, telemetry);
        robot.rotateToGyroHeading(0, 0.3, telemetry);
    }
}