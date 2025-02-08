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
    public static PIDCoefficientsEx SLIDE_PID_COEFFICIENTS = new PIDCoefficientsEx(
            0.0125, 0.7, 0.005, 0, 0, 0
    );
    public static FeedforwardCoefficientsEx SLIDE_FEEDFORWARD_COEFFICIENTS = new FeedforwardCoefficientsEx(
            0, 0, 0, 0.1, 0.0
    );
    public static double target = 0;
    @Override
    public void runOpMode() {
        OuttakeSlide outtakeSlide = new OuttakeSlide(this);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        outtakeSlide.init();
        //slide.setTarget(target);
        while (opModeIsActive()) {
            //if (gamepad1.dpad_up) target = EXTEND;
            //if (gamepad1.dpad_down) target = RETRACT;
            //if (gamepad1.square) {
            //    slide.setPIDCoef(SLIDE_PID_COEFFICIENTS);
            //    slide.setFFCoef(SLIDE_FEEDFORWARD_COEFFICIENTS);
            //    slide.setTarget(target);
            //}

            //boolean endLoop = slide.loop();

            //telemetry.addData("target", slide.target);
            telemetry.addData("pos", outtakeSlide.getPos());
            //telemetry.addData("ok", endLoop);
            telemetry.update();
        }
    }
}
