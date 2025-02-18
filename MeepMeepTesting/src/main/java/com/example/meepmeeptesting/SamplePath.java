package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SamplePath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(39, 62, Math.toRadians(180.00)))
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
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}