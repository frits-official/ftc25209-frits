package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constant.ARM;

public class Arm {
    private DcMotorEx leftArmMotor, rightArmMotor;
    private LinearOpMode opMode;
    private PIDEx controller;
    private double targetAngle;

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
    }

    public void setPIDCoef(PIDCoefficientsEx coef) {
        controller = new PIDEx(coef);
    }

    public void setPower(double power) {
        leftArmMotor.setPower(power);
        rightArmMotor.setPower(power);
    }

    public double getPower() {
        return leftArmMotor.getPower();
    }
    public double getPos() { return leftArmMotor.getCurrentPosition(); }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public void updateMotorPosition() {
        if (Math.abs(targetAngle - getPos()) > ARM.ANGLE_TOLERANCE) {
            double power = controller.calculate(targetAngle, getPos());
            setPower(-power);
        } else setPower(0);
    }
}
