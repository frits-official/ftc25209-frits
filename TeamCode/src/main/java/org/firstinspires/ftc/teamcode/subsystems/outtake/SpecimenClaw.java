package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constant;

public class SpecimenClaw {
    private LinearOpMode opMode;
    public ServoImplEx specimenServo;

    public SpecimenClaw(LinearOpMode _opMode) {
        this.opMode = _opMode;
    }

    public void init() {
        specimenServo = opMode.hardwareMap.get(ServoImplEx.class, "specimenServo");
        specimenServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        
        grab();
    }

    public void grab() {
        specimenServo.setPosition(Constant.CLAW.GRAB);
    }

    public void release() {
        specimenServo.setPosition(Constant.CLAW.RELEASE);
    }

    public boolean isGrab() {
        return (specimenServo.getPosition() == 1);
    }
}
