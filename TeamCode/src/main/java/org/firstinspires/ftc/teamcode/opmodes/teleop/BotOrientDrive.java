package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.Utility.sense;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constant.*;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Arm;
import org.firstinspires.ftc.teamcode.subsystems.intake.Claw;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Slide;

@TeleOp(group = "TeleOp")
public class BotOrientDrive extends LinearOpMode {
    private Drivetrain drive;
    private Arm arm;
    private Claw claw;
    private Slide slide;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain(this.hardwareMap);
        arm = new Arm(this);
        claw = new Claw(this);
        slide = new Slide(this);

        waitForStart();

        if (isStopRequested()) return;

        arm.init();
        claw.init();
        slide.init();

        while (opModeIsActive() && !isStopRequested()) {
            drive.setWeightedDrivePower(new Pose2d(
                    sense(-gamepad1.left_stick_y, GAMEPAD_SENSITIVITY.SENSE_Y) * SPEED.STRAIGHT_SPEED,
                    sense(-gamepad1.left_stick_x, GAMEPAD_SENSITIVITY.SENSE_X) * SPEED.STRAFE_SPEED,
                    sense(-gamepad1.right_stick_x, GAMEPAD_SENSITIVITY.SENSE_Z) * SPEED.TURN_SPEED
            ));

            claw.control();
            arm.setPower(-gamepad2.left_stick_y);
            slide.setPower(-gamepad2.right_stick_y);

            telemetry.addData("arm power", arm.getPower());
            telemetry.addData("slide power", slide.getPower());
            telemetry.update();
        }
    }
}
