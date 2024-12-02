package org.firstinspires.ftc.teamcode.opmodes.Test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.SampleMecanumDrive;

@TeleOp(group = "Test")
public class TestIMU extends LinearOpMode {
    private SampleMecanumDrive drivetrain;
    @Override
    public void runOpMode() {
        drivetrain = new SampleMecanumDrive(this.hardwareMap);

        waitForStart();
        if (opModeInInit()) {
            drivetrain.imu.resetYaw();
            while (opModeIsActive()) {
                YawPitchRollAngles robotOrientation = drivetrain.imu.getRobotYawPitchRollAngles();

                telemetry.addData("Yaw", robotOrientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Pitch", robotOrientation.getPitch(AngleUnit.DEGREES));
                telemetry.addData("Roll", robotOrientation.getRoll(AngleUnit.DEGREES));
                telemetry.update();
            }
        }
    }
}
