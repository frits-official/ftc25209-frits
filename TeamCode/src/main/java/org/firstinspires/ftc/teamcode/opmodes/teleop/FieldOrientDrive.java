package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.Constant.ARM.GRAB_ANGLE;
import static org.firstinspires.ftc.teamcode.Constant.ARM.TRANSFER_ANGLE;
import static org.firstinspires.ftc.teamcode.Constant.SLIDE.RETRACT;
import static org.firstinspires.ftc.teamcode.Utility.enableBulkRead;
import static org.firstinspires.ftc.teamcode.Utility.sense;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constant.GAMEPAD_SENSITIVITY;
import org.firstinspires.ftc.teamcode.Constant.SPEED;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Arm;
import org.firstinspires.ftc.teamcode.subsystems.intake.Claw;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Slide;

@Config
@TeleOp(group = "TeleOp", name = "cam dung cai nay")
public class FieldOrientDrive extends LinearOpMode {
    private Drivetrain drive;
    private Arm arm;
    private Claw claw;
    private Slide slide;
    private Servo dumper;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain(this.hardwareMap);
        arm = new Arm(this);
        claw = new Claw(this);
        slide = new Slide(this);
        dumper = hardwareMap.get(Servo.class, "dumperServo");

        arm.init();
        claw.init();
        slide.init();

        arm.setTargetAngle(TRANSFER_ANGLE);
        slide.setTarget(RETRACT);
        dumper.setPosition(0);

        enableBulkRead(this.hardwareMap);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.driveFieldCentric(
                    sense(gamepad1.left_stick_y, GAMEPAD_SENSITIVITY.SENSE_Y) * SPEED.STRAIGHT_SPEED,
                    sense(gamepad1.left_stick_x, GAMEPAD_SENSITIVITY.SENSE_X) * SPEED.STRAFE_SPEED,
                    sense(gamepad1.right_stick_x, GAMEPAD_SENSITIVITY.SENSE_Z) * SPEED.TURN_SPEED
            );

            if (gamepad1.left_bumper) {
                claw.gripperServo.setPosition(1);
                sleep(400);
                arm.setTargetAngle(TRANSFER_ANGLE);
                claw.wristServo.setPosition(1);
            }

            if (gamepad1.right_bumper) {
                claw.gripperServo.setPosition(0.4);
                claw.wristServo.setPosition(0);
                arm.setTargetAngle(GRAB_ANGLE);
            }

            if (gamepad1.square) {
                claw.gripperServo.setPosition(0.4);
                sleep(400);
                claw.wristServo.setPosition(0);
                arm.setTargetAngle(GRAB_ANGLE);
            }

            if (gamepad1.cross) dumper.setPosition(0);

            if (gamepad1.circle) dumper.setPosition(1);

            claw.control();
            arm.loop();
            //slide.manualControl(go);

            telemetry.addData("arm power", arm.getPower());
            telemetry.addData("arm target", arm.targetAngle);
            telemetry.addData("arm angle", arm.getAngle());
            telemetry.addLine();
            telemetry.addData("slide power", slide.getPower());
            telemetry.addData("slide pos", slide.getCurrentPosition());
            telemetry.update();
        }
    }
}
