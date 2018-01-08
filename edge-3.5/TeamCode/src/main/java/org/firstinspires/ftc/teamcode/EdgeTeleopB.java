package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Teleop Mode B")
//@Disabled
public class EdgeTeleopB extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Declaring the hardware object
    EdgeBot robot;

    double constant = 180;

    @Override
    public void runOpMode() {
        // Initializing the hardware object
        robot = new EdgeBot();
        robot.init(hardwareMap, this);

        // Wait for the driver to press play
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            robot.waitForTick(10);

            // Scale and set motor values
            double forwardSpeed = gamepad1.left_stick_y;

            if (Math.abs(forwardSpeed) < 0.5) {
                forwardSpeed *= 0.5;
            }

            robot.mecanumDrive2(gamepad1.left_stick_x, forwardSpeed, gamepad1.right_stick_x, constant);

            if (gamepad1.a) {
                constant += 10;
                robot.waitForTick(100);
            } else if (gamepad1.b) {
                constant -= 10;
                robot.waitForTick(100);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Constant", constant);
            telemetry.update();
        }
    }
}
