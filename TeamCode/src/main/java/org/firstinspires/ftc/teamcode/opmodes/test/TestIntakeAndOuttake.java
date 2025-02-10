package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeWrist;

@TeleOp(group = "Test")
public class TestIntakeAndOuttake extends LinearOpMode {
    @Override
    public void runOpMode() {
        IntakeSlide intakeSlide = new IntakeSlide(this);
        IntakeWrist intakeWrist = new IntakeWrist(this);
        IntakeClaw intakeClaw = new IntakeClaw(this);
        OuttakeSlide outtakeSlide = new OuttakeSlide(this);
        OuttakeWrist outtakeWrist = new OuttakeWrist(this);
        OuttakeClaw outtakeClaw = new OuttakeClaw(this);
        outtakeSlide.init();
        outtakeWrist.init();
        outtakeClaw.init();
        intakeSlide.init();
        intakeWrist.init();
        intakeClaw.init();
        waitForStart();
        while (opModeIsActive()) {
            if (-gamepad1.left_stick_y > 0) intakeSlide.setPosition(0);
            if (-gamepad1.left_stick_y < 0) {
                intakeSlide.setPosition(0.43);
                intakeWrist.retract();
            }

            if (gamepad1.triangle) intakeWrist.extend();
            if (gamepad1.cross) intakeWrist.retract();

            if (gamepad1.square) intakeClaw.grab();
            if (gamepad1.circle) intakeClaw.release();

            if (gamepad1.dpad_left) intakeClaw.rotateClaw(36);
            if (gamepad1.dpad_right) intakeClaw.rotateClaw(120);
            if (gamepad1.dpad_up) intakeClaw.rotateClaw(0);
            if (gamepad1.dpad_down) intakeClaw.rotateClaw(90);


            if (gamepad1.left_bumper) {
                intakeClaw.release();
                sleep(350);
                outtakeWrist.bucket();
                outtakeClaw.bucket();
            }
            if (gamepad1.right_bumper) {
                outtakeClaw.release();
                outtakeWrist.transfer();
                outtakeClaw.transfer();
            }

            if (gamepad1.left_trigger > 0) outtakeClaw.grab();
            if (gamepad1.right_trigger > 0) outtakeClaw.release();

            telemetry.addData("slide pos", intakeSlide.getPosition());
            telemetry.addData("claw wrist", intakeWrist.getPosition());
            telemetry.addData("is grab", intakeClaw.isGrab());
            telemetry.addData("rotate angle", intakeClaw.getClawAngle());
            telemetry.update();
        }
    }
}

