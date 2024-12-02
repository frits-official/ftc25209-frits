package org.firstinspires.ftc.teamcode.opmodes.Test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.SampleMecanumDrive;

@TeleOp
public class TestDrivetrain extends LinearOpMode {
    private SampleMecanumDrive drivetrain;

    @Override
    public void runOpMode() {
        drivetrain = new SampleMecanumDrive(this.hardwareMap);

        waitForStart();
        if (opModeInInit()) {
            drivetrain.imu.resetYaw();
            while (opModeIsActive()) {
                YawPitchRollAngles robotOrientation = drivetrain.imu.getRobotYawPitchRollAngles();

                Vector2d input = new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ).rotated(-robotOrientation.getYaw(AngleUnit.RADIANS));

                drivetrain.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x
                        )
                );
            }
        }
    }
}
