package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWrist;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.outtake.OuttakeWrist;

@Autonomous(group = "Auto")
public class AutoSamplePath extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drivetrain drive = new Drivetrain(hardwareMap);

        IntakeSlide intakeSlide = new IntakeSlide(this);
        IntakeWrist intakeWrist = new IntakeWrist(this);
        IntakeClaw intakeClaw = new IntakeClaw(this);

        OuttakeSlide outtakeSlide = new OuttakeSlide(this);
        OuttakeWrist outtakeWrist = new OuttakeWrist(this);
        OuttakeClaw outtakeClaw = new OuttakeClaw(this);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeSlide.init();
        outtakeWrist.init();
        outtakeClaw.init();

        intakeSlide.init();
        intakeWrist.init();
        intakeClaw.init();

        TrajectorySequence path = drive.trajectorySequenceBuilder(new Pose2d(39, 62, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(55, 55, Math.toRadians(-135)))
                .waitSeconds(5)
                //score sample pid
                .splineToLinearHeading(new Pose2d(48, 40, Math.toRadians(-90)), Math.toRadians(-135))
                .waitSeconds(3)
                //get sample
                .lineToSplineHeading(new Pose2d(55, 55, Math.toRadians(-135)))
                .waitSeconds(5)
                //score sample pid
                .splineToLinearHeading(new Pose2d(58, 40, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(3)
                //get sample
                .lineToSplineHeading(new Pose2d(55, 55, Math.toRadians(-135)))
                .waitSeconds(5)
                //score sample pid
                .build();
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(path);
    }
}
