package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.nio.file.attribute.PosixFileAttributes;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        DefaultBotBuilder defaultBotBuilder = new DefaultBotBuilder(meepMeep);
        defaultBotBuilder.setConstraints(65, 65, Math.toRadians(180), Math.toRadians(180), 15);// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        RoadRunnerBotEntity myBot = defaultBotBuilder
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(12,- 60, Math.toRadians(90)))
                                .waitSeconds(0.250)


                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}