package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constant.SPEED;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DistanceSensorLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

@TeleOp(group = "Test")
public class TestDrivetrainBotCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drive = new Drivetrain(hardwareMap);
        DistanceSensorLocalizer localizer = new DistanceSensorLocalizer(this);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        localizer.init();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y * SPEED.STRAIGHT_SPEED,
                    -gamepad1.left_stick_x * SPEED.STRAFE_SPEED,
                    -gamepad1.right_stick_x * SPEED.TURN_SPEED
            ));

            telemetry.addData("left", localizer.getLeftDis());
            telemetry.addData("right", localizer.getRightDis());
            telemetry.addData("rear", localizer.getRearDis());
            telemetry.update();
        }
    }
}