package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.outtake.Slide;

@TeleOp(group = "Test")
public class TestMotor extends LinearOpMode {
    @Override
    public void runOpMode() {
        Slide slide = new Slide(this);

        waitForStart();
        slide.init();
        while (opModeIsActive()) {
            if (gamepad1.square) slide.resetEncoder();
            slide.setPower(-gamepad1.left_stick_y + 0.1);

            telemetry.addData("pos", slide.getPos());
            telemetry.update();
        }
    }
}
