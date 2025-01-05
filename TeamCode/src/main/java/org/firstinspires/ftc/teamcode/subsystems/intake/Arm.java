package org.firstinspires.ftc.teamcode.subsystems.intake;

import static org.firstinspires.ftc.teamcode.Constant.ARM.*;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.Estimator;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.ThermalEquilibrium.homeostasis.Utils.WPILibMotionProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

public class Arm {
    private DcMotorEx leftArmMotor, rightArmMotor;
    private LinearOpMode opMode;
    private Double targetAngle = null;

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

    private WPILibMotionProfile motionProfile = null;
    private ElapsedTime timer = new ElapsedTime();
    public WPILibMotionProfile.State targetState;

    public Arm(LinearOpMode linearOpMode) {
        this.opMode = linearOpMode;
    }

    public void init() {
        leftArmMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "leftArmMotor");
        rightArmMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "rightArmMotor");
        leftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
        leftArmMotor.setPower(power);
        rightArmMotor.setPower(power);
    }

    public double getPower() { return rightArmMotor.getPower(); }
    public double getCurrentPosition() { return rightArmMotor.getCurrentPosition(); }
    public double getAngle() { return filter.update() * DEG_PER_TICK + TICK_OFFSET; }

    public void setTargetAngle(Double _target) {
        this.targetAngle = _target;
        pid = new PIDEx(ARM_PID_COEFFICIENTS);
        if (targetAngle != null)
            motionProfile = new WPILibMotionProfile(
                    ARM_VA_CONSTRAINT,
                    new WPILibMotionProfile.State(targetAngle, 0),
                    new WPILibMotionProfile.State(getAngle(), 0));
        timer.reset();
    }

    public void resetPID() {
        pid = new PIDEx(ARM_PID_COEFFICIENTS);
    }

    public boolean loop() {
        if (targetAngle == null) {
            pid.calculate(0, getAngle());
            return true;
        }
        WPILibMotionProfile.State targetState = motionProfile.calculate(timer.seconds());
        this.targetState = targetState;
        double pidPow = pid.calculate(targetState.position, getAngle());
        if (Math.abs(getAngle() - targetAngle) < ANGLE_TOLERANCE) {
            setPower(0);
            return true;
        }

        double ffPow = feedforward.calculate(
                Math.toRadians(targetState.position - getAngle()),
                targetState.velocity,
                0);
        if (timer.seconds() > motionProfile.totalTime())
            ffPow = 0;
        setPower((ffPow + pidPow) * 12 / batteryVoltageSensor.getVoltage());
        return false;
    }
}
