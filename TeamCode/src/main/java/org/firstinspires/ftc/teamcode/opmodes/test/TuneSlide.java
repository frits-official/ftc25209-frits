package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.Constant.VER_SLIDE.EXTEND;
import static org.firstinspires.ftc.teamcode.Constant.VER_SLIDE.RETRACT;

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
public class TuneSlide extends LinearOpMode {
    public static PIDCoefficientsEx SLIDE_PID_COEFFICIENTS = new PIDCoefficientsEx(
            0.005, 0, 0.0, 0, 0, 0
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
        outtakeSlide.setTarget(target);
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) target = EXTEND;
            if (gamepad1.dpad_down) target = RETRACT;
            if (gamepad1.square) {
                outtakeSlide.setPIDCoef(SLIDE_PID_COEFFICIENTS);
                outtakeSlide.setFFCoef(SLIDE_FEEDFORWARD_COEFFICIENTS);
                outtakeSlide.setTarget(target);
            }

            boolean endLoop = outtakeSlide.loop();

            telemetry.addData("target", outtakeSlide.target);
            telemetry.addData("pos", outtakeSlide.getPos());
            telemetry.addData("ok", endLoop);
            telemetry.update();
        }
    }
}
