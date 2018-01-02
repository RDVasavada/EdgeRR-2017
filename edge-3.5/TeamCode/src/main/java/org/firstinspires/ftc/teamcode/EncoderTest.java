package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Encoder test")
//@Disabled
public class EncoderTest extends LinearOpMode {

    // The hardware object
    EdgeBot robot;

    // Timer
    ElapsedTime timer;

    @Override
    public void runOpMode() {
        // Initialize the hardware object
        robot = new EdgeBot();
        robot.init(hardwareMap);

        robot.rotateClockwiseEncoder(90, 0.3, telemetry);
    }
}