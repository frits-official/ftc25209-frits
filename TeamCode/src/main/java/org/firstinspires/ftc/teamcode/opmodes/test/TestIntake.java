package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWrist;

@TeleOp(group = "Test")
public class TestIntake extends LinearOpMode {
    @Override
    public void runOpMode() {
        IntakeSlide slide = new IntakeSlide(this);
        IntakeWrist wrist = new IntakeWrist(this);
        IntakeClaw claw = new IntakeClaw(this);
        slide.init();
        wrist.init();
        claw.init();
        waitForStart();
        while (opModeIsActive()) {
            slide.setPosition(-gamepad1.left_stick_y + slide.getPosition());

            if (gamepad1.triangle) wrist.extend();
            if (gamepad1.cross) wrist.retract();

            if (gamepad1.square) claw.grab();
            if (gamepad1.circle) claw.release();

            if (gamepad1.dpad_left) claw.rotateClaw(45);
            if (gamepad1.dpad_up) claw.rotateClaw(90);
            if (gamepad1.dpad_down) claw.rotateClaw(135);

            telemetry.addData("slide pos", slide.getPosition());
            telemetry.addData("claw wrist", wrist.getPosition());
            telemetry.addData("is grab", claw.isGrab());
            telemetry.addData("rotate angle", claw.getClawAngle());
            telemetry.update();
        }
    }
}
