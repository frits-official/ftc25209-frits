package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SpecimenPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-26, 62, Math.toRadians(0.00)))
                        //.waitSeconds(2)
                        .splineToSplineHeading(new Pose2d(-36, 37, Math.toRadians(-90)), Math.toRadians(-135))
                        .lineTo(new Vector2d(-36, 14))
                        .lineTo(new Vector2d(-42, 14))
                        .lineTo(new Vector2d(-48, 55))
                        .lineTo(new Vector2d(-36, 14))
                        .lineTo(new Vector2d(-52, 14))
                        .lineTo(new Vector2d(-52, 55))
                        .lineTo(new Vector2d(-52, 14))
                        .lineTo(new Vector2d(-60, 14))
                        .lineTo(new Vector2d(-60, 55))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}