package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constant;

public class OuttakeWrist {
    private LinearOpMode opMode;
    public ServoImplEx leftWristServo, rightWristServo;

    public OuttakeWrist(LinearOpMode _opMode) {
        this.opMode = _opMode;
    }

    public void init() {
        leftWristServo = opMode.hardwareMap.get(ServoImplEx.class, "leftOuttakeWristServo");
        rightWristServo = opMode.hardwareMap.get(ServoImplEx.class, "rightOuttakeWristServo");
        leftWristServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightWristServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        transfer();
    }

    public void setPosition(double pos) {
        pos = Math.max(Math.min(pos, 1), -1);
        leftWristServo.setPosition(pos);
        rightWristServo.setPosition(1 - pos);
    }

    public void transfer() {
        setPosition(Constant.VER_SLIDE.WRIST.TRANSFER);
    }

    public void bucket() {
        setPosition(Constant.VER_SLIDE.WRIST.BUCKET);
    }

    public double getPosition() {
        return leftWristServo.getPosition();
    }
}
