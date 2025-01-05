package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.intake.Arm;

@Config
@TeleOp(group = "Test")
public class TestArm extends LinearOpMode {
    @Override
    public void runOpMode() {
        Arm arm = new Arm(this);
        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
