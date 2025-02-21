package org.firstinspires.ftc.teamcode.subsystems.outtake;

import static org.firstinspires.ftc.teamcode.Constant.VER_SLIDE.*;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

public class OuttakeSlide {
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

    public ElapsedTime timer = new ElapsedTime();
    public OuttakeSlide(LinearOpMode linearOpMode) {
        this.opMode = linearOpMode;
    }

    public void init() {
        leftSlideMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlideMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "rightSlide");
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

    public double getPower() {
        return rightSlideMotor.getPower();
    }

    public double getCurrentPosition() {
        return rightSlideMotor.getCurrentPosition();
    }

    public double getPos() {
        return filter.update();
    }

    public void manualControl(boolean go, double input) {
        double power = 0;
        if ((input > 0) && (getPos() < EXTEND) && go) power = input;
        else if ((input < 0)) power = input;
        setPower(power + 0.1);
    }

    public void setTarget(Double _target) {
        setPIDCoef(SLIDE_PID_COEFFICIENTS);
        setFFCoef(SLIDE_FEEDFORWARD_COEFFICIENTS);
        this.target = _target;
        timer.reset();
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

        if (timer.milliseconds() > 1500) return true;

        double ffPow = feedforward.calculate(
                target - getPos(),
                0,
                0);
        setPower((ffPow + pidPow) * 12 / batteryVoltageSensor.getVoltage());
        return false;
    }
}
