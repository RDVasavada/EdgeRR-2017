package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.*;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.IOException;


@TeleOp(name="DogeCV Red Cryptobox Detector", group="DogeCV")
//@Disabled

public class DogeCVCryptoBoxTest extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private EdgeBot robot;

    private CryptoboxDetector cryptoboxDetector = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void runOpMode() {
        cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        robot = new EdgeBot();
        robot.init(hardwareMap, this);

        cryptoboxDetector.rotateMat = true;

        //Optional Test Code to load images via Drawables
        //cryptoboxDetector.useImportedImage = true;
        //cryptoboxDetector.SetTestMat(com.qualcomm.ftcrobotcontroller.R.drawable.test_cv4);

        cryptoboxDetector.enable();

        waitForStart();

        robot.closeClampServos();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("isCryptoBoxDetected", cryptoboxDetector.isCryptoBoxDetected());
            telemetry.addData("isColumnDetected ",  cryptoboxDetector.isColumnDetected());

            telemetry.addData("Column Left ",  cryptoboxDetector.getCryptoBoxLeftPosition());
            telemetry.addData("Column Center ",  cryptoboxDetector.getCryptoBoxCenterPosition());
            telemetry.addData("Column Right ",  cryptoboxDetector.getCryptoBoxRightPosition());

            double error = cryptoboxDetector.getCryptoBoxCenterPosition() - 350;

            if (error > 10) {
                while (error > 10 && opModeIsActive() && gamepad1.a) {
                    robot.driveForwards(0.1);
                    error = cryptoboxDetector.getCryptoBoxCenterPosition() - 350;
                }
            } else if (error < -10) {
                while (error < -10 && opModeIsActive() && gamepad1.a) {
                    robot.driveBackwards(0.1);
                    error = cryptoboxDetector.getCryptoBoxCenterPosition() - 350;
                }
            } else {
                if (gamepad1.a) {
                    robot.stopDriveMotors();

                    cryptoboxDetector.disable();

                    robot.rotateClockwiseEncoder(90, 0.3, telemetry);
                    runtime.reset();
                    while (runtime.seconds() < 5 && opModeIsActive()) {
                        robot.driveForwards(0.3);
                    }
                    robot.openClampServos();

                    stop();
                }
            }

            telemetry.addData("Error", error);
            telemetry.update();
        }
    }

}