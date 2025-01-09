package org.firstinspires.ftc.teamcode.opmodes.test;

import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Utils.WPILibMotionProfile;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.intake.Arm;

@Config
@TeleOp(group = "Test")
public class TestArm extends LinearOpMode {
    public static PIDCoefficientsEx ARM_PID_COEFFICIENTS = new PIDCoefficientsEx(
            0.11, 0.5, 0.05, 0, 0, 0
    );
    public static FeedforwardCoefficientsEx ARM_FEEDFORWARD_COEFFICIENTS = new FeedforwardCoefficientsEx(
            0, 0, 0, 0, 0.0001
    );
    public static double target = 0;
    @Override
    public void runOpMode() {
        Arm arm = new Arm(this);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        arm.init();
        while (opModeIsActive()) {
            if (gamepad1.square) {
                arm.setPIDCoef(ARM_PID_COEFFICIENTS);
                arm.setFFCoef(ARM_FEEDFORWARD_COEFFICIENTS);
                arm.setTargetAngle(target);
            }

            boolean endLoop = arm.loop();

            telemetry.addData("target", arm.targetAngle);
            telemetry.addData("angle", arm.getAngle());
            telemetry.addData("ok", endLoop);
            telemetry.update();
        }
    }
}
