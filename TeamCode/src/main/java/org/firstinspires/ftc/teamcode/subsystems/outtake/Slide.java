package org.firstinspires.ftc.teamcode.subsystems.outtake;

import static org.firstinspires.ftc.teamcode.Constant.SLIDE.*;

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

public class Slide {
    private DcMotorEx leftSlideMotor, rightSlideMotor;
    private LinearOpMode opMode;
    private Double target = null;

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

    private WPILibMotionProfile motionProfile = null;
    private ElapsedTime timer = new ElapsedTime();
    public WPILibMotionProfile.State targetState;

    public Slide(LinearOpMode linearOpMode) {
        this.opMode = linearOpMode;
    }

    public void init() {
        leftSlideMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightSlideMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
        // TODO: change direction
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

    public double getPower() { return rightSlideMotor.getPower(); }
    public double getCurrentPosition() { return rightSlideMotor.getCurrentPosition(); }
    public double getTick() { return filter.update(); }

    public void setTarget(Double _target) {
        this.target = _target;
        pid = new PIDEx(SLIDE_PID_COEFFICIENTS);
        if (target != null)
            motionProfile = new WPILibMotionProfile(
                    SLIDE_VA_CONSTRAINT,
                    new WPILibMotionProfile.State(target, 0),
                    new WPILibMotionProfile.State(getTick(), 0));
        timer.reset();
    }

    public void resetPID() {
        pid = new PIDEx(SLIDE_PID_COEFFICIENTS);
    }

    public boolean loop() {
        if (target == null) {
            pid.calculate(0, getTick());
            return true;
        }
        WPILibMotionProfile.State targetState = motionProfile.calculate(timer.seconds());
        this.targetState = targetState;
        double pidPow = pid.calculate(targetState.position, getTick());
        if (Math.abs(getTick() - target) < TOLERANCE) {
            setPower(0);
            return true;
        }

        double ffPow = feedforward.calculate(
                targetState.position - getTick(),
                targetState.velocity,
                0);
        if (timer.seconds() > motionProfile.totalTime())
            ffPow = 0;
        setPower((ffPow + pidPow) * 12 / batteryVoltageSensor.getVoltage());
        return false;
    }
}
