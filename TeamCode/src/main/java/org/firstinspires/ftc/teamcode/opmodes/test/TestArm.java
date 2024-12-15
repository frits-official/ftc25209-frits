package org.firstinspires.ftc.teamcode.opmodes.test;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.intake.Arm;

@Config
@TeleOp (group = "Test")
public class TestArm extends LinearOpMode {
    private Arm arm;
    public static double p = 0, i = 0, d = 0;
    public static double maxISum = 0, stabilityThresh = 0, lowPassGain = 0;
    public static double target = 0;
    @Override
    public void runOpMode() {
        arm = new Arm(this);

        waitForStart();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm.init();
        while (opModeIsActive()) {
            arm.setPIDCoef(new PIDCoefficientsEx(p, i, d, maxISum, stabilityThresh, lowPassGain));
            arm.setTargetAngle(target);
            arm.updateMotorPosition();

            telemetry.addData("power", arm.getPower());
            telemetry.addData("target angle", target);
            telemetry.addData("raw angle", arm.potentiometer.getRawAngle());
            telemetry.addData("filtered angle", arm.potentiometer.getFilteredAngle());
            telemetry.update();
        }
    }
}
