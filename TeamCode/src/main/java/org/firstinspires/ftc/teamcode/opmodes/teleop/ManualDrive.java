package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constant.SPEED;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWrist;

@TeleOp(group = "TeleOp")
public class ManualDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drive = new Drivetrain(hardwareMap);
        IntakeSlide slide = new IntakeSlide(this);
        IntakeWrist wrist = new IntakeWrist(this);
        IntakeClaw claw = new IntakeClaw(this);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.init();
        wrist.init();
        claw.init();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y * SPEED.STRAIGHT_SPEED,
                    -gamepad1.left_stick_x * SPEED.STRAFE_SPEED,
                    -gamepad1.right_stick_x * SPEED.TURN_SPEED
            ));

            if (gamepad1.left_bumper) slide.setPosition(0);
            if (gamepad1.right_bumper) { slide.setPosition(0.43); wrist.retract(); }

            if (gamepad1.triangle) wrist.extend();
            if (gamepad1.cross) wrist.retract();

            if (gamepad1.square) claw.grab();
            if (gamepad1.circle) claw.release();

            if (gamepad1.dpad_left) claw.rotateClaw(40);
            if (gamepad1.dpad_right) claw.rotateClaw(120);
            if (gamepad1.dpad_up) claw.rotateClaw(0);
            if (gamepad1.dpad_down) claw.rotateClaw(90);

            telemetry.addData("x", -gamepad1.left_stick_y);
            telemetry.addData("y", -gamepad1.left_stick_x);
            telemetry.addData("z", -gamepad1.right_stick_x);

            telemetry.addData("slide pos", slide.getPosition());
            telemetry.addData("claw wrist", wrist.getPosition());
            telemetry.addData("is grab", claw.isGrab());
            telemetry.addData("rotate angle", claw.getClawAngle());
            telemetry.update();
        }
    }
}