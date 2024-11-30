package org.firstinspires.ftc.teamcode.opmodes.Test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(group = "Test")
public class TestIMU extends LinearOpMode {
    private IMU imu;
    @Override
    public void runOpMode() {
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

                telemetry.addData("Yaw", robotOrientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Pitch", robotOrientation.getPitch(AngleUnit.DEGREES));
                telemetry.addData("Roll", robotOrientation.getRoll(AngleUnit.DEGREES));
                telemetry.update();
            }
        }
    }
}
