package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Teleop Mode With Intake")
//@Disabled
public class EdgeTeleop2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Declaring the hardware object
    EdgeBot2 robot;

    // Boolean for lift motor
    boolean liftMotorSlack;

    @Override
    public void runOpMode() {
        // Initializing the hardware object
        robot = new EdgeBot2();
        robot.init(hardwareMap, this);

        liftMotorSlack = false;

        // Wait for the driver to press play
        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {

            // Update the boolean
            if (gamepad1.dpad_up) {
                liftMotorSlack = false;
            } else if (gamepad1.dpad_down) {
                liftMotorSlack = true;
            }

            // Scale  motor values
            double forwardSpeed = gamepad1.left_stick_y;
            double strafeSpeed = gamepad1.left_stick_x;

            if (Math.abs(forwardSpeed) < 0.2) {
                forwardSpeed *= 0.25;
            }

            if (Math.abs(strafeSpeed) < 0.2) {
                strafeSpeed *= 0.25;
            }

            // Set motor values
            robot.mecanumDrive(strafeSpeed, forwardSpeed, gamepad1.right_stick_x, telemetry);

            // Update the lift motor
            if (gamepad1.right_trigger > 0.2) {
                robot.raiseLiftMotor();
                telemetry.addData("Lift ", "up");
            } else if (gamepad1.left_trigger > 0.2) {
                robot.lowerLiftMotor();
                telemetry.addData("Lift ", "down");
            } else if (!liftMotorSlack){
                robot.stopLiftMotor();
            } else if (liftMotorSlack) {
                robot.slackLiftMotor();
            }

            // Update the intake motor
            if (gamepad2.left_bumper) {
                robot.intakeIn();
                telemetry.addData("Intake ", "in");
            } else if (gamepad2.right_bumper) {
                robot.intakeOut();
                telemetry.addData("Intake ", "out");
            } else if (gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_up || gamepad2.dpad_right) {
                robot.intakeStop();
            }

            // Update the intake servos
            if (gamepad1.a) {
                robot.closeIntakeServos();
                telemetry.addData("Clamp servos ", "closing");
            } else if (gamepad1.b) {
                robot.openIntakeServosHalfway();
                telemetry.addData("Clamp servos ", "opening halfway");
            } else if (gamepad1.y) {
                robot.openIntakeServos();
                telemetry.addData("Clamp servos ", "open fully");
            }

            // Update the crane rotation servo
            if (Math.abs(gamepad2.left_stick_x) > 0.1) {
                robot.craneRotate(gamepad2.left_stick_x);
                telemetry.addData("Crane ", "rotating");
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
            } else if (gamepad2.a) {
                robot.clawWristHalfway();
                telemetry.addData("wrist ", "halfway");
            } else if (gamepad2.b) {
                robot.clawWristLow();
                telemetry.addData("wrist ", "halfway (lower)");
            }

            // Update the crane motor
            if (gamepad2.left_trigger > 0.2) {
                robot.craneMotorBackward();
                telemetry.addData("Crane motor ", "back");
            } else if (gamepad2.right_trigger > 0.2) {
                robot.craneMotorForward();
                telemetry.addData("Crane motor ", "forward");
            } else {
                robot.stopCraneMotor();
            }

            // Update the claw pinch servo
            if (gamepad2.y) {
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
