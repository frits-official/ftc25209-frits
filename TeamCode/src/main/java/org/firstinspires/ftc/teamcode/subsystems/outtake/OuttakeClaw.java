package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constant;

public class OuttakeClaw {
    private LinearOpMode opMode;
    public ServoImplEx outtakeClawServo, outtakeWristServo;

    public OuttakeClaw(LinearOpMode _opMode) {
        this.opMode = _opMode;
    }

    public void init() {
        outtakeClawServo = opMode.hardwareMap.get(ServoImplEx.class, "outtakeClawServo");
        outtakeWristServo = opMode.hardwareMap.get(ServoImplEx.class, "outtakeWristServo");
        outtakeClawServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        outtakeWristServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        bucket();
        release();
    }

    public void grab() {
        outtakeClawServo.setPosition(Constant.CLAW.GRAB);
    }

    public void release() {
        outtakeClawServo.setPosition(Constant.CLAW.RELEASE);
    }

    public void rotateJoint(double angle) {
        outtakeWristServo.setPosition(angle);
    }

    public void transfer() {
        rotateJoint(Constant.VER_SLIDE.JOINT.TRANSFER);
    }

    public void bucket() {
        rotateJoint(Constant.VER_SLIDE.JOINT.BUCKET);
    }


    public double getClawAngle() {
        return outtakeWristServo.getPosition();
    }

    public boolean isGrab() {
        return (outtakeClawServo.getPosition() == 1);
    }
}
