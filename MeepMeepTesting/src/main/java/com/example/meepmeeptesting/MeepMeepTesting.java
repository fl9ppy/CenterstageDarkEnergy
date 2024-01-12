package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        //PRELOADo
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -59, 90))
                                .lineToLinearHeading(new Pose2d(-44, -24, Math.toRadians(0)))
                                //INTAKE
                                .lineToLinearHeading(new Pose2d(-62,-11, Math.toRadians(0)))
                                //OUTAKE
                                .lineToLinearHeading(new Pose2d(44, -11, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(44,-35, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(44, -11, Math.toRadians(0)))
                                //INTAKE2
                                .lineToLinearHeading(new Pose2d(-62,-11, Math.toRadians(0)))
                                //OUTAKE2
                                .lineToLinearHeading(new Pose2d(44, -11, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(44,-35, Math.toRadians(0)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}