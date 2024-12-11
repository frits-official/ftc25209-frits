package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

@TeleOp(group = "Test")
public class TestIMU extends LinearOpMode {
    private Drivetrain drivetrain;
    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(this.hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
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
