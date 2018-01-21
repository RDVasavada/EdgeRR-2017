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


@TeleOp(name="DogeCV Glyph Detector", group="DogeCV")
@Disabled
public class DogeCVGlyphTest extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private EdgeBot robot;


    private GlyphDetector glyphDetector = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");


        glyphDetector = new GlyphDetector();
        glyphDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        glyphDetector.minScore = 1;
        glyphDetector.downScaleFactor = 0.3;
        glyphDetector.speed = GlyphDetector.GlyphDetectionSpeed.SLOW;
        glyphDetector.enable();

        robot = new EdgeBot();
        robot.init(hardwareMap, this);

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            while(runtime.seconds() < 3) {
                telemetry.addData("Setting ", "up");
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Glyph Pos X", glyphDetector.getChosenGlyphOffset());
            telemetry.addData("Glyph Pos Offest", glyphDetector.getChosenGlyphPosition().toString());

            telemetry.update();

            if (gamepad1.a) {
                if (glyphDetector.getChosenGlyphPosition().y > 10) {
                    robot.driveForwardForInches(glyphDetector.getChosenGlyphPosition().y / 20, 0.3);
                } else {
                    robot.closeClampServos();
                }
            }
        }
    }
}