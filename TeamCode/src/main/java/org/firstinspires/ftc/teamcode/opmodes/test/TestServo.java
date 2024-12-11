package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (group = "Test")
public class TestServo extends LinearOpMode {
    private Servo armServo1, armServo2;

    @Override
    public void runOpMode() {
        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        armServo2 = hardwareMap.get(Servo.class, "armServo2");

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

                telemetry.addData("Servo 1 pos (deg)", armServo1.getPosition() * 180);
                telemetry.addData("Servo 2 pos (deg)", armServo2.getPosition() * 180);

                telemetry.update();
            }
        }
    }
}
