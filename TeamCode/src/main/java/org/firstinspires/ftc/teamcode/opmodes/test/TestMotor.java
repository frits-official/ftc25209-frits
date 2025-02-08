package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSlide;

@TeleOp(group = "Test")
public class TestMotor extends LinearOpMode {
    @Override
    public void runOpMode() {
        //retract 0.38; extend 0
        DcMotor motor = hardwareMap.get(DcMotor.class, "leftFront");
        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(-gamepad1.left_stick_y);
            telemetry.addData("pos", -gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
