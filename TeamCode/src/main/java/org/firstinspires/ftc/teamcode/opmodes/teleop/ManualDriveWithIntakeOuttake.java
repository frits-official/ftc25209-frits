package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeWrist;

@TeleOp(group = "TeleOp")
public class ManualDriveWithIntakeOuttake extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drivetrain drive = new Drivetrain(hardwareMap);
        IntakeSlide intakeSlide = new IntakeSlide(this);
        IntakeWrist intakeWrist = new IntakeWrist(this);
        IntakeClaw intakeClaw = new IntakeClaw(this);
        OuttakeSlide outtakeSlide = new OuttakeSlide(this);
        OuttakeWrist outtakeWrist = new OuttakeWrist(this);
        OuttakeClaw outtakeClaw = new OuttakeClaw(this);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeSlide.init();
        outtakeWrist.init();
        outtakeClaw.init();
        intakeSlide.init();
        intakeWrist.init();
        intakeClaw.init();
        waitForStart();
        while (opModeIsActive()) {
            // drivetrain
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y * Constant.SPEED.STRAIGHT_SPEED,
                    -gamepad1.left_stick_x * Constant.SPEED.STRAFE_SPEED,
                    -gamepad1.right_stick_x * Constant.SPEED.TURN_SPEED
            ));
            // extend horizontal slide
            if (-gamepad2.left_stick_y > 0) {
                intakeClaw.release();
                intakeSlide.setPosition(0);
            }
            // retract horizontal slide
            if (-gamepad2.left_stick_y < 0) {
                intakeSlide.setPosition(0.43);
                intakeWrist.retract();
                intakeClaw.rotateClaw(90);
                outtakeWrist.transfer();
                outtakeClaw.transfer();
            }
            // extend/retract intake wrist
            if (gamepad2.cross) intakeWrist.extend();
            if (gamepad2.triangle) intakeWrist.retract();
            // grab from ground
            if (gamepad2.square || gamepad1.x) intakeClaw.grab();
            if (gamepad2.circle || gamepad1.b) intakeClaw.release();
            // rotate intake claw
            if (gamepad1.dpad_right) intakeClaw.rotateClaw(45);
            if (gamepad1.dpad_left) intakeClaw.rotateClaw(135);
            if (gamepad1.dpad_up) intakeClaw.rotateClaw(0);
            if (gamepad1.dpad_down) intakeClaw.rotateClaw(90);
            // go to bucket
            if (gamepad2.dpad_up) {
                intakeClaw.release();
                sleep(350);
                outtakeWrist.bucket();
                outtakeClaw.bucket();
            }
            // go to transfer
            if (gamepad2.dpad_down) {
                outtakeClaw.release();
                outtakeWrist.transfer();
                outtakeClaw.transfer();
            }
            // outtake claw
            if (gamepad2.left_trigger > 0) outtakeClaw.grab();
            if (gamepad2.right_trigger > 0) outtakeClaw.release();

            outtakeSlide.manualControl(true);

            telemetry.addData("intake slide pos", intakeSlide.getPosition());
            telemetry.addData("intake claw wrist", intakeWrist.getPosition());
            telemetry.addData("intake is grab", intakeClaw.isGrab());
            telemetry.addData("intake rotate angle", intakeClaw.getClawAngle());
            telemetry.addLine();
            telemetry.addData("outtake slide pos", outtakeSlide.getPos());
            telemetry.addData("outtake claw wrist", outtakeWrist.getPosition());
            telemetry.addData("outtake is grab", outtakeClaw.isGrab());
            telemetry.addData("outtake rotate angle", outtakeClaw.getClawAngle());
            telemetry.update();
        }
    }
}

