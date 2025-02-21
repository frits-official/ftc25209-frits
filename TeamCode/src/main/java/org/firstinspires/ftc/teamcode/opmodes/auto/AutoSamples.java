package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.Utility;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DistanceSensorLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeWrist;

@Autonomous(group = "Auto")
public class AutoSamples extends LinearOpMode {
    Drivetrain drive;
    IntakeSlide intakeSlide;
    IntakeWrist intakeWrist;
    IntakeClaw intakeClaw;
    OuttakeSlide outtakeSlide;
    OuttakeWrist outtakeWrist;
    OuttakeClaw outtakeClaw;
    DistanceSensorLocalizer localizer;
    ElapsedTime timer = new ElapsedTime();

    // remember, all is cm
    public void forward(double target) {
        timer.reset();
        double curr = localizer.getRearDis();
        while (timer.seconds() <= 6 && localizer.getRearDis() < (target + curr)) {
            drive.goForward(0.5);
        }
    }
    public void backward(double target) {
        timer.reset();
        double curr = localizer.getRearDis();
        while (timer.seconds() <= 6 && localizer.getRearDis() > (-target + curr)) {
            drive.goForward(-0.5);
        }
    }
    public void goLeft(double target) {
        timer.reset();
        double curr = localizer.getLeftDis();
        while (timer.seconds() <= 10 && localizer.getLeftDis() < (target + curr)) {
            drive.strafeLeft(0.5);
        }
    }
    public void goRight(double target) {
        timer.reset();
        double curr = localizer.getRightDis();
        while (timer.seconds() <= 6 && localizer.getRightDis() < (target + curr)) {
            drive.strafeRight(0.5);
        }
    }

    public void goSlide(double target) {
        outtakeSlide.setTarget(target);
        while (!outtakeSlide.loop()) {
            telemetry.addData("pos", outtakeSlide.getPos());
            telemetry.update();
        };
    }

    public void out() {
        outtakeSlide.setPower(0.1);
        outtakeWrist.bucket();
        outtakeClaw.bucket();
        sleep(800);
        outtakeClaw.release();
        sleep(500);
        outtakeWrist.transfer();
        outtakeClaw.transfer();
    }

    @Override
    public void runOpMode() {
        drive = new Drivetrain(hardwareMap);

        intakeSlide = new IntakeSlide(this);
        intakeWrist = new IntakeWrist(this);
        intakeClaw = new IntakeClaw(this);

        outtakeSlide = new OuttakeSlide(this);
        outtakeWrist = new OuttakeWrist(this);
        outtakeClaw = new OuttakeClaw(this);

        localizer = new DistanceSensorLocalizer(this);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeSlide.init();
        outtakeWrist.init();
        outtakeClaw.init();

        intakeSlide.init();
        intakeWrist.init();
        intakeClaw.init();

        localizer.init();

        Utility.enableBulkRead(this.hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        //remember, all is cm
        forward(45);
        drive.turn(Math.toRadians(-50));
        goSlide(2200.0);
        backward(7);
        sleep(500);
        out();
        forward(3);
        goSlide(4);
        drive.turn(Math.toRadians(50));
        backward(2);
        while (opModeIsActive()) {
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y * Constant.SPEED.STRAIGHT_SPEED,
                    -gamepad1.left_stick_x * Constant.SPEED.STRAFE_SPEED,
                    -gamepad1.right_stick_x * Constant.SPEED.TURN_SPEED
            ));
        }
    }
}
