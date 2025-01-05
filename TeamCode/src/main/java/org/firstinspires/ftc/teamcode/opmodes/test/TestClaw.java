package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.intake.Arm;
import org.firstinspires.ftc.teamcode.subsystems.intake.Claw;

@TeleOp(group = "Test")
public class TestClaw extends LinearOpMode {
    @Override
    public void runOpMode() {
        Claw claw = new Claw(this);
        Arm arm = new Arm(this);

        waitForStart();
        arm.init();
        claw.init();
        while (opModeIsActive()) {
            claw.control();
            arm.setPower(-gamepad1.left_stick_y);
        }
    }
}
