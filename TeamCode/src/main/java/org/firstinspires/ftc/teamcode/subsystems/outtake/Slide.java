package org.firstinspires.ftc.teamcode.subsystems.outtake;

import static org.firstinspires.ftc.teamcode.Constant.SLIDE.*;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.Estimator;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.function.DoubleSupplier;

public class Slide {
    private DcMotorEx leftSlideMotor, rightSlideMotor;
    private LinearOpMode opMode;
    public Double target = null;

    private VoltageSensor batteryVoltageSensor;
    private PIDEx pid = new PIDEx(SLIDE_PID_COEFFICIENTS);
    private FeedforwardEx feedforward = new FeedforwardEx(SLIDE_FEEDFORWARD_COEFFICIENTS);

    private DoubleSupplier encoder = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return getCurrentPosition();
        }
    };
    private Estimator filter = new KalmanEstimator(encoder, KALMAN_GAIN.Q, KALMAN_GAIN.R, KALMAN_GAIN.N);

    public Slide(LinearOpMode linearOpMode) {
        this.opMode = linearOpMode;
    }

    public void init() {
        leftSlideMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightSlideMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
        leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
    }

    public void resetEncoder() {
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }

    public double getPower() { return leftSlideMotor.getPower(); }
    public double getCurrentPosition() { return leftSlideMotor.getCurrentPosition(); }
    public double getPos() { return filter.update(); }

    public void manualControl() {
        setPower((-opMode.gamepad2.right_stick_y + 0.1) * 12 / batteryVoltageSensor.getVoltage());
        if (opMode.gamepad2.square) resetEncoder();
    }

    public void setTarget(Double _target) {
        setPIDCoef(SLIDE_PID_COEFFICIENTS);
        setFFCoef(SLIDE_FEEDFORWARD_COEFFICIENTS);
        this.target = _target;
    }

    public void setPIDCoef(PIDCoefficientsEx coef) {
        pid = new PIDEx(coef);
    }
    public void setFFCoef(FeedforwardCoefficientsEx coef) {
        feedforward = new FeedforwardEx(coef);
    }

    public boolean loop() {
        if (target == null) {
            pid.calculate(0, getPos());
            return true;
        }
        double pidPow = pid.calculate(target, getPos());
        if (Math.abs(getPos() - target) < TOLERANCE) {
            setPower(0);
            return true;
        }

        double ffPow = feedforward.calculate(
                Math.toRadians(target - getPos()),
                0,
                0);
        setPower((ffPow + pidPow) * 12 / batteryVoltageSensor.getVoltage());
        return false;
    }
}
