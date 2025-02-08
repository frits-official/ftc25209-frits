package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constant;

public class IntakeSlide {
    private LinearOpMode opMode;
    public ServoImplEx leftServo, rightServo;

    public IntakeSlide(LinearOpMode _opMode) {
        this.opMode = _opMode;
    }

    public void init() {
        leftServo = opMode.hardwareMap.get(ServoImplEx.class, "leftServo");
        rightServo = opMode.hardwareMap.get(ServoImplEx.class, "rightServo");
        leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        setPosition(1);
    }

    public void setPosition(double pos) {
        pos = Math.max(Math.min(pos, 1), -1);
        leftServo.setPosition(pos * Constant.HOR_SLIDE.MAX_POS);
        rightServo.setPosition((1 - pos) * Constant.HOR_SLIDE.MAX_POS);
    }

    public double getPosition() {
        return leftServo.getPosition() / 0.38;
    }
}
