package org.firstinspires.ftc.teamcode.opmodes.Test;

import static org.firstinspires.ftc.teamcode.Constant.VISION.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.Camera;
import org.firstinspires.ftc.teamcode.subsystems.vision.DetectSampleProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

@TeleOp(group = "Test")
public class TestColor extends LinearOpMode {
    private Camera camera = new Camera(CAMERA_NAME, this);
    private DetectSampleProcessor sampleProcessor = new DetectSampleProcessor(camera, ColorRange.RED);
    @Override
    public void runOpMode() {
        sampleProcessor.initSampleProcessor();
        camera.buildCamera(CAMERA_RESOLUTION);

        waitForStart();
        while(opModeIsActive() || opModeInInit()) {
            sampleProcessor.process();
            for (double angle: sampleProcessor.getAngles()) {
                telemetry.addData("angle: ", angle);
            }
            telemetry.update();
        }
    }
}
