package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.*;


@Autonomous(name="Rotation Test")
@Disabled
public class RotationTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Declaring the hardware object
    EdgeBot robot;

    @Override
    public void runOpMode() {
        // Initializing the hardware object
        robot = new EdgeBot();
        robot.init(hardwareMap);

        // Wait for the driver to press play
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            robot.rotateClockwiseEncoder(90, 0.2, telemetry);

            stop();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
