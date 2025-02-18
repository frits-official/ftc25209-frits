package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constant;

public class IntakeWrist {
    private LinearOpMode opMode;
    public ServoImplEx leftWristServo, rightWristServo;

    public IntakeWrist(LinearOpMode _opMode) {
        this.opMode = _opMode;
    }

    public void init() {
        leftWristServo = opMode.hardwareMap.get(ServoImplEx.class, "leftWristServo");
        rightWristServo = opMode.hardwareMap.get(ServoImplEx.class, "rightWristServo");
        leftWristServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightWristServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        setPwmEnable();
        retract();
    }

    public void setPosition(double pos) {
        setPwmEnable();
        pos = Math.max(Math.min(pos, 1), -1);
        leftWristServo.setPosition(pos);
        rightWristServo.setPosition(1 - pos);
    }

    public void extend() {
        setPosition(Constant.HOR_SLIDE.WRIST_EXTEND_POS);
    }

    public void retract() {
        setPosition(Constant.HOR_SLIDE.WRIST_RETRACT_POS);
    }

    public double getPosition() {
        return leftWristServo.getPosition();
    }

    public void setPwmEnable() {
        leftWristServo.setPwmEnable();
        rightWristServo.setPwmEnable();
    }
    public void setPwmDisable() {
        leftWristServo.setPwmDisable();
        rightWristServo.setPwmDisable();
    }
}
