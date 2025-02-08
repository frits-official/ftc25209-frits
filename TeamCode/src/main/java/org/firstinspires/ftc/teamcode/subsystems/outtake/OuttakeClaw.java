package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeClaw {
    private LinearOpMode opMode;
    public Servo gripServo, firstOuttakeWristServo, leftSecondWristServo, rightSecondWristServo;
    public OuttakeClaw(LinearOpMode _opMode) {
        this.opMode = _opMode;
    }
    public void init() {
        gripServo = opMode.hardwareMap.get(Servo.class, "outtakeGrip");
        firstOuttakeWristServo = opMode.hardwareMap.get(Servo.class, "firstOuttakeWrist");
        leftSecondWristServo = opMode.hardwareMap.get(Servo.class, "leftSecondOuttakeWrist");
        rightSecondWristServo = opMode.hardwareMap.get(Servo.class, "rightSecondOuttakeWrist");
    }
    public void setSecondWristPosition(double pos) {
        leftSecondWristServo.setPosition(pos);
        rightSecondWristServo.setPosition(1 - pos);
    }
}
