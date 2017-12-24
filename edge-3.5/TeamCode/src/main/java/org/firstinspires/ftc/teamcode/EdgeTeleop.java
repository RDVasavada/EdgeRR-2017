package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Teleop Mode")
//@Disabled
public class EdgeTeleop extends LinearOpMode {

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

            // Scale and set motor values
            double forwardSpeed = gamepad1.left_stick_y;

            if (Math.abs(gamepad1.left_stick_y) < 0.5) {
                forwardSpeed *= 0.5;
            }

            robot.mecanumDrive(gamepad1.left_stick_x, forwardSpeed, gamepad1.right_stick_x, telemetry);

            // Update the clamp servos
            if (gamepad1.a) {
                robot.closeClampServos();
                telemetry.addData("Clamp servos ", "closing");
            } else if (gamepad1.b) {
                robot.openClampServos(0.5);
                telemetry.addData("Clamp servos ", "opening halfway");
            } else if (gamepad1.y) {
                robot.openClampServos(1);
                telemetry.addData("Clamp servos ", "open fully");
            }

            // Update the crane rotation servo
            if (gamepad2.left_stick_x < -0.1) {
                robot.craneRotateLeft();
                telemetry.addData("Crane ", "rotating left");
            } else if (gamepad2.left_stick_x > 0.1) {
                robot.craneRotateRight();
                telemetry.addData("Crane ", "rotating right");
            } else {
                robot.craneRotateStop();
            }

            // Update the claw wrist servo
            if (gamepad2.right_stick_y > 0.1) {
                robot.clawWristDown();
                telemetry.addData("wrist ", "moving down");
            } else if (gamepad2.right_stick_y < -0.1) {
                robot.clawWristUp();
                telemetry.addData("wrist ", "moving up");
            }

            // Update the lift motor
            if (gamepad2.right_trigger > 0.2) {
                robot.raiseLiftMotor(gamepad2.right_trigger);
                telemetry.addData("Lift ", "up");
            } else if (gamepad2.left_trigger > 0.2) {
                robot.lowerLiftMotor(gamepad2.left_trigger);
                telemetry.addData("Lift ", "down");
            } else {
                robot.stopLiftMotor();
            }

            // Update the crane motor
            if (gamepad2.a) {
                robot.craneMotorBackward();
                telemetry.addData("Crane motor ", "back");
            } else if (gamepad2.y) {
                robot.craneMotorForward();
                telemetry.addData("Crane motor ", "forward");
            } else {
                robot.stopCraneMotor();
            }

            // Update the claw pinch servo
            if (gamepad2.b) {
                robot.clawPinch();
                telemetry.addData("claw ", "pinching");
            } else if (gamepad2.x) {
                robot.clawOpen();
                telemetry.addData("claw ", "opening");
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
