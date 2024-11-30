package org.firstinspires.ftc.teamcode.opmodes.Test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.SampleMecanumDrive;

@TeleOp
public class TestDrivetrain extends LinearOpMode {
    private SampleMecanumDrive drivetrain;
    private IMU imu;

    @Override
    public void runOpMode() {
        drivetrain = new SampleMecanumDrive(this.hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        )));
        waitForStart();
        if (opModeInInit()) {
            imu.resetYaw();
            while (opModeIsActive()) {
                YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();

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
