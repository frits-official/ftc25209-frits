package org.firstinspires.ftc.teamcode.opmodes.Test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(group = "Test")
public class TestCustomMecanum extends LinearOpMode {
    private DcMotor leftFront, rightFront;
    private DcMotor leftRear, rightRear;
    private IMU imu;
    private boolean isFieldCentric = false;
    private YawPitchRollAngles robotOrientation;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));

        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Circle to enable field centric, square to disable.");

            if (gamepad1.x) isFieldCentric = true;
            if (gamepad1.b) isFieldCentric = false;
            telemetry.addData("isFieldCentric", isFieldCentric);

            double rotY = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double rotX = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            if (isFieldCentric) {
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                rotX = rotX * Math.cos(-botHeading) - rotX * Math.sin(-botHeading);
                rotY = rotX * Math.sin(-botHeading) + rotY * Math.cos(-botHeading);
            }

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            telemetry.update();
        }
    }
}
