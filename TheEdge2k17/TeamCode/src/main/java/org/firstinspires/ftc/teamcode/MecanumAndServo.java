package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.math.*;

@TeleOp(name="Mecanum Wheel and Servo Test")
//@Disabled
public class MecanumAndServo extends LinearOpMode {

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

            // Set motor values
            robot.mecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, telemetry);

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

            // Update the clamp servos
            if (gamepad1.right_trigger > 0.1) {
                robot.openClampServos(gamepad1.right_trigger);
                telemetry.addData("Clamp servos ", "open");
            } else {
                robot.closeClampServos();
            }

            // Update the crane motor
            if (gamepad1.a) {
                robot.craneMotorBackward();
                telemetry.addData("Crane motor ", "back");
            } else if (gamepad1.y) {
                robot.craneMotorForward();
                telemetry.addData("Crane motor ", "forward");
            } else {
                robot.stopCraneMotor();
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
            } else {
                robot.clawWristStop();
            }

            // Update the claw pinch servo
            if (gamepad2.b) {
                robot.clawPinch();
                telemetry.addData("claw ", "pinching");
            } else if (gamepad2.x) {
                robot.clawOpen();
                telemetry.addData("claw ", "opening");
            } else {
                robot.clawPinchStop();
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
