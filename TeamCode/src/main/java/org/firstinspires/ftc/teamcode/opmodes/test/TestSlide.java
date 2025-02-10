package org.firstinspires.ftc.teamcode.opmodes.test;

import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSlide;

@TeleOp(group = "Test")
public class TestSlide extends LinearOpMode {
    @Override
    public void runOpMode() {
        OuttakeSlide outtakeSlide = new OuttakeSlide(this);
        waitForStart();
        outtakeSlide.init();
        while (opModeIsActive()) {
            outtakeSlide.setPower(-gamepad1.left_stick_y + 0.1);

            telemetry.addData("vel", outtakeSlide.getPower());
            telemetry.addData("pos", outtakeSlide.getPos());
            telemetry.update();
        }
    }
}
