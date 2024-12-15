package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constant.SPEED;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

@TeleOp(group = "Test")
public class TestDrivetrainFieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drive = new Drivetrain(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.driveFieldCentric(
                    gamepad1.left_stick_y * SPEED.STRAIGHT_SPEED,
                    gamepad1.left_stick_x * SPEED.STRAFE_SPEED,
                    gamepad1.right_stick_x * SPEED.TURN_SPEED
            );
        }
    }
}