package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeWrist;

@Config
@TeleOp(group = "Test")
public class TestOuttake extends LinearOpMode {
    public static double wristPos = 0;
    public static double angle = 0;
    @Override
    public void runOpMode() {
        OuttakeSlide slide = new OuttakeSlide(this);
        OuttakeWrist wrist = new OuttakeWrist(this);
        OuttakeClaw claw = new OuttakeClaw(this);
        slide.init();
        wrist.init();
        claw.init();
        waitForStart();
        while (opModeIsActive()) {
            wrist.setPosition(wristPos);
            claw.rotateJoint(angle);

            if (gamepad1.square) claw.grab();
            if (gamepad1.circle) claw.release();

            telemetry.addData("claw wrist", wrist.getPosition());
            telemetry.addData("is grab", claw.isGrab());
            telemetry.addData("rotate angle", claw.getClawAngle());
            telemetry.update();
        }
    }
}

