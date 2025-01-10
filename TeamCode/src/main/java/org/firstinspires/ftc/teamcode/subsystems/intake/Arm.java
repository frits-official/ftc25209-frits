package org.firstinspires.ftc.teamcode.subsystems.intake;

import static org.firstinspires.ftc.teamcode.Constant.ARM.*;

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

public class Arm {
    private DcMotorEx leftArmMotor, rightArmMotor;
    private LinearOpMode opMode;
    public Double targetAngle = null;

    private VoltageSensor batteryVoltageSensor;
    private PIDEx pid = new PIDEx(ARM_PID_COEFFICIENTS);
    private FeedforwardEx feedforward = new FeedforwardEx(ARM_FEEDFORWARD_COEFFICIENTS);

    private DoubleSupplier encoder = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return getCurrentPosition();
        }
    };
    private Estimator filter = new KalmanEstimator(encoder, KALMAN_GAIN.Q, KALMAN_GAIN.R, KALMAN_GAIN.N);

    public Arm(LinearOpMode linearOpMode) {
        this.opMode = linearOpMode;
    }

    public void init() {
        leftArmMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "leftArmMotor");
        rightArmMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "rightArmMotor");
        rightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
    }

    public void resetEncoder() {
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        double limitedPower = Math.min(Math.abs(power), MAX_VELO) * Math.signum(power);

        leftArmMotor.setPower(limitedPower);
        rightArmMotor.setPower(limitedPower);
    }

    public double getPower() { return rightArmMotor.getPower(); }
    public double getCurrentPosition() { return rightArmMotor.getCurrentPosition(); }
    public double getAngle() { return getCurrentPosition() * DEG_PER_TICK + TICK_OFFSET; }

    public void setTargetAngle(Double _target) {
        this.targetAngle = _target;
        //pid = new PIDEx(ARM_PID_COEFFICIENTS);
    }

    public void manualControl() {
        double power = 0;
        if (opMode.gamepad1.left_bumper && (getAngle() < TRANSFER_ANGLE)) power = 1;
        else if (opMode.gamepad1.right_bumper && (getAngle() > GRAB_ANGLE)) power = -1;
        double ffPow = feedforward.calculate(
                Math.toRadians(getAngle()),
                0,
                0);
        setPower(power + ffPow);
    }

    public void resetPID() {
        pid = new PIDEx(ARM_PID_COEFFICIENTS);
    }
    public void setPIDCoef(PIDCoefficientsEx coef) {
        pid = new PIDEx(coef);
    }
    public void setFFCoef(FeedforwardCoefficientsEx coef) {
        feedforward = new FeedforwardEx(coef);
    }

    public boolean loop() {
        if (targetAngle == null) {
            pid.calculate(0, getAngle());
            return true;
        }
        double pidPow = pid.calculate(targetAngle, getAngle());
        if (Math.abs(getAngle() - targetAngle) < ANGLE_TOLERANCE) {
            setPower(0);
            return true;
        }

        double ffPow = feedforward.calculate(
                Math.toRadians(getAngle()),
                0,
                0);
        setPower((ffPow + pidPow) * 12 / batteryVoltageSensor.getVoltage());
        return false;
    }
}
