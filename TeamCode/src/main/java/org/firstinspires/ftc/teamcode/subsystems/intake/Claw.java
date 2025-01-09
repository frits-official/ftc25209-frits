package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public Servo gripperServo, wristServo;
    private LinearOpMode opMode;

    public Claw(LinearOpMode _opMode) { this.opMode = _opMode; }

    public void init() {
        gripperServo = opMode.hardwareMap.get(Servo.class, "gripperServo");
        wristServo = opMode.hardwareMap.get(Servo.class, "wristServo");

        gripperServo.setPosition(1);
        wristServo.setPosition(0);
    }

    public void control() {
        // control claw
        if (opMode.gamepad1.dpad_left) gripperServo.setPosition(0.4); // release
        if (opMode.gamepad1.dpad_right) gripperServo.setPosition(1); // grip
        //control wrist
        if (opMode.gamepad1.dpad_down) wristServo.setPosition(0); // point to ground
        if (opMode.gamepad1.dpad_up) wristServo.setPosition(1); // point to basket
    }
}
