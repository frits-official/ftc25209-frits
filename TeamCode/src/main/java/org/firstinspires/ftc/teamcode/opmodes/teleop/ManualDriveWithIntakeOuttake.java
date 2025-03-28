package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.Utility;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.outtake.SpecimenClaw;

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
        SpecimenClaw specimenClaw = new SpecimenClaw(this);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeSlide.init();
        outtakeSlide.resetEncoder();
        outtakeWrist.init();
        outtakeClaw.init();
        specimenClaw.init();

        intakeSlide.init();
        intakeWrist.init();
        intakeClaw.init();

        double slidePower = 0;
        boolean slidePowerLock = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // drivetrain
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y * Constant.SPEED.STRAIGHT_SPEED,
                    -gamepad1.left_stick_x * Constant.SPEED.STRAFE_SPEED,
                    -gamepad1.right_stick_x * Constant.SPEED.TURN_SPEED
            ));
            // extend horizontal slide
            if (Utility.sense(-gamepad2.left_stick_y, Constant.HOR_SLIDE.SENSE) > 0) {
                //intakeClaw.release();
                intakeSlide.setPosition(0);
                outtakeWrist.bucket();
                outtakeClaw.bucket();
            }
            // retract horizontal slide
            if (Utility.sense(-gamepad2.left_stick_y, Constant.HOR_SLIDE.SENSE) < 0) {
                intakeSlide.setPosition(0.43);
                intakeWrist.retract();
                intakeClaw.rotateClaw(90);
                outtakeClaw.release();
                sleep(300);
                outtakeWrist.transfer();
                outtakeClaw.transfer();
            }
            // extend/retract intake wri st
            if (gamepad2.a || gamepad2.cross) intakeWrist.semiExtend();
            if (gamepad2.y || gamepad2.triangle) intakeWrist.retract();
            // grab from ground
            if (gamepad2.x || gamepad1.square || gamepad2.square) {
                //if (intakeWrist.getPosition() == Constant.HOR_SLIDE.WRIST_EXTEND_POS) intakeWrist.setPwmDisable();
                if (intakeWrist.getPosition() == Constant.HOR_SLIDE.WRIST_RETRACT_POS) intakeClaw.grab();
                else {
                    intakeWrist.extend();
                    sleep(100);
                    intakeClaw.grab();
                    sleep(200);
                    intakeWrist.semiExtend();
                }
            }
            if (gamepad2.b || gamepad1.circle || gamepad2.circle) {
                intakeClaw.release();
                intakeWrist.setPwmEnable();
                intakeWrist.setPosition(intakeWrist.getPosition());
            }
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
            // go get specimen
            //if (gamepad2.dpad_left) {
            //    outtakeClaw.release();
            //    outtakeWrist.getSpecimen();
            //    outtakeClaw.getSpecimen();
            //}
            // go get off specimen
            //if (gamepad2.dpad_right) {
            //    outtakeWrist.getOffSpecimen();
            //    outtakeClaw.getOffSpecimen();
            //}
            // outtake claw
            if (gamepad2.left_trigger > 0) outtakeClaw.grab();
            if (gamepad2.right_trigger > 0) outtakeClaw.release();
            /*
            if (gamepad2.left_bumper) {
                slidePower = -gamepad2.right_stick_y;
                slidePowerLock = true;
            }
            if (gamepad2.right_bumper) {
                slidePowerLock = false;
            }

            if (!slidePowerLock) outtakeSlide.manualControl((!intakeClaw.isGrab()) && (outtakeClaw.isGrab()), -gamepad2.right_stick_y);
            else outtakeSlide.manualControl((!intakeClaw.isGrab()) && (outtakeClaw.isGrab()), slidePower);
            */
            outtakeSlide.manualControl((!intakeClaw.isGrab()) && (outtakeClaw.isGrab()) || gamepad2.left_bumper, -gamepad2.right_stick_y);
            if (gamepad1.left_bumper) outtakeSlide.resetEncoder();

            if (gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) specimenClaw.grab();
            if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) specimenClaw.release();

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

