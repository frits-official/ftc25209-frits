package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.intake.Arm;

@TeleOp(group = "Test")
public class ResetArm extends LinearOpMode {
    @Override
    public void runOpMode() {
        Arm arm = new Arm(this);

        waitForStart();
        arm.init();
        while (opModeIsActive()) {
            if (gamepad1.square) arm.resetEncoder();
            telemetry.addData("pos", arm.getAngle());
            telemetry.update();
        }
    }
}
