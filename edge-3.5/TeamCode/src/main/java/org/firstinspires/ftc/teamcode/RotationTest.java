package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Rotation test")
@Disabled
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

        robot.rotateCounterClockwiseEncoder(90, 0.3, telemetry);

        timer.reset();

        while (timer.seconds() < 2) {

        }

        robot.rotateCounterClockwiseEncoder(90, 0.3, telemetry);

        while (timer.seconds() < 2) {

        }

        robot.rotateClockwiseEncoder(90, 0.3, telemetry);

        while (timer.seconds() < 2) {

        }

        robot.rotateClockwiseEncoder(90, 0.3, telemetry);
    }
}