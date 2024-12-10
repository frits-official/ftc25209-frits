package org.firstinspires.ftc.teamcode.opmodes.Test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.SampleMecanumDrive;

@TeleOp (group = "Test")
public class TestDrivetrain extends LinearOpMode {
    private SampleMecanumDrive drivetrain;
    private boolean isFieldCentric = false;
    private YawPitchRollAngles robotOrientation;

    @Override
    public void runOpMode() {
        drivetrain = new SampleMecanumDrive(this.hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
            drivetrain.imu.resetYaw();
            while (opModeIsActive()) {
                telemetry.addLine("Circle to enable field centric, square to disable.");

                if (gamepad1.circle || gamepad1.x) isFieldCentric = true;
                else if (gamepad1.square || gamepad1.b) isFieldCentric = false;
                telemetry.addData("isFieldCentric", isFieldCentric);

                if (isFieldCentric) {
                    robotOrientation = drivetrain.imu.getRobotYawPitchRollAngles();

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
                } else {
                    drivetrain.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    gamepad1.left_stick_x,
                                    gamepad1.right_stick_x
                            )
                    );
                }

                telemetry.update();
            }
        }
    }
}
