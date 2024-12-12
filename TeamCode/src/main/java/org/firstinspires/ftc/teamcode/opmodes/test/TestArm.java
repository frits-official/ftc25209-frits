package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (group = "Test")
public class TestArm extends LinearOpMode {
    private Servo armServo1, armServo2;
    private DcMotor armMotor1, armMotor2;

    @Override
    public void runOpMode() {
        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        armServo2 = hardwareMap.get(Servo.class, "armServo2");
        armMotor1 = hardwareMap.get(DcMotor.class, "armMotor1");
        armMotor2 = hardwareMap.get(DcMotor.class, "armMotor2");
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (opModeIsActive()) {
            armServo1.setPosition(0);
            armServo2.setPosition(0);
            while (opModeIsActive()) {
                telemetry.addLine("R1/R2 to control servo 1, L1/L2 to control servo 2");

                if (gamepad1.right_bumper) armServo1.setPosition(armServo1.getPosition() + 1.0 / 180.0);
                else if (gamepad1.right_trigger > 0) armServo1.setPosition(armServo1.getPosition() - 1.0 / 180.0);

                if (gamepad1.left_bumper) armServo2.setPosition(armServo2.getPosition() + 1.0 / 180.0);
                else if (gamepad1.left_trigger > 0) armServo2.setPosition(armServo2.getPosition() - 1.0 / 180.0);

                // arm1: -9 6
                // arm2: -82 76

                if (gamepad1.dpad_up) {
                    armMotor1.setTargetPosition(-82);
                    armMotor2.setTargetPosition(76);
                    armMotor1.setPower(0.8); armMotor2.setPower(0.8);
                } else if (gamepad1.dpad_down) {
                    armMotor1.setTargetPosition(-9);
                    armMotor2.setTargetPosition(6);
                    armMotor1.setPower(0.8); armMotor2.setPower(0.8);
                }

                telemetry.addData("Servo 1 pos (deg)", armServo1.getPosition() * 180);
                telemetry.addData("Servo 2 pos (deg)", armServo2.getPosition() * 180);
                telemetry.addData("Motor count 1", armMotor1.getCurrentPosition());
                telemetry.addData("Motor count 2", armMotor2.getCurrentPosition());

                telemetry.update();
            }
        }
    }
}
