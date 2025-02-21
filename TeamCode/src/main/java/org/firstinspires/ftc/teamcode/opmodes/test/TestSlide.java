package org.firstinspires.ftc.teamcode.opmodes.test;

import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSlide;

@Config
@TeleOp(group = "Test")
public class TestSlide extends LinearOpMode {
    public static double target = 0;
    @Override
    public void runOpMode() {
        OuttakeSlide outtakeSlide = new OuttakeSlide(this);
        waitForStart();
        outtakeSlide.init();
        outtakeSlide.setTarget(target);
        while (opModeIsActive()) {
            if (gamepad1.circle) outtakeSlide.resetEncoder();
            if (gamepad1.square) outtakeSlide.setTarget(target);
            boolean res = outtakeSlide.loop();

            telemetry.addData("is done", res);
            telemetry.addData("time", outtakeSlide.timer.milliseconds());
            telemetry.addData("vel", outtakeSlide.getPower());
            telemetry.addData("pos", outtakeSlide.getPos());
            telemetry.update();
        }
    }
}
