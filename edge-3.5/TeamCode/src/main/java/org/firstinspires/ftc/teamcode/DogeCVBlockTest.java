package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="DogeCV Block Detector", group="DogeCV")
//@Disabled

public class DogeCVBlockTest extends LinearOpMode
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
        robot = new EdgeBot();
        robot.init(hardwareMap, this);

        glyphDetector = new GlyphDetector();
        glyphDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        glyphDetector.minScore = 1;
        glyphDetector.downScaleFactor = 0.3;
        glyphDetector.speed = GlyphDetector.GlyphDetectionSpeed.SLOW;
        glyphDetector.enable();

        waitForStart();

        robot.closeClampServos();
        robot.phoneOut();

        runtime.reset();

        while (opModeIsActive()) {

            if (runtime.seconds() > 2) {

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Glyph Pos X", glyphDetector.getChosenGlyphOffset());
                telemetry.addData("Glyph Pos Offest", glyphDetector.getChosenGlyphPosition());

                telemetry.update();
            }
        }
    }

}
