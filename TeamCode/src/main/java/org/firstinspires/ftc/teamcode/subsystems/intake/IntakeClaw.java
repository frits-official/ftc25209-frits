package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constant;

public class IntakeClaw {
    private LinearOpMode opMode;
    public ServoImplEx intakeClawServo, intakeWristServo;

    public IntakeClaw(LinearOpMode _opMode) {
        this.opMode = _opMode;
    }

    public void init() {
        intakeClawServo = opMode.hardwareMap.get(ServoImplEx.class, "intakeClawServo");
        intakeWristServo = opMode.hardwareMap.get(ServoImplEx.class, "intakeWristServo");
        intakeClawServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeWristServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void grab() {
        intakeClawServo.setPosition(Constant.CLAW.GRAB);
    }

    public void release() {
        intakeClawServo.setPosition(Constant.CLAW.RELEASE);
    }

    /**
     * @param angle from 0 to 180 deg
     */
    public void rotateClaw(double angle) {
        intakeWristServo.setPosition(angle / 180);
    }

    public double getClawAngle() {
        return intakeWristServo.getPosition() * 180;
    }

    public boolean isGrab() {
        return (intakeClawServo.getPosition() == 1);
    }
}
