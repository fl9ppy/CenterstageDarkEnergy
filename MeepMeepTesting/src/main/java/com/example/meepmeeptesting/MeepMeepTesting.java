package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.nio.file.attribute.PosixFileAttributes;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(90), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                                //preload
                                .lineToLinearHeading(new Pose2d(-40,-30, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-30, -28, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-40,-30, Math.toRadians(0)))
                                //pedrum
                                .splineToLinearHeading(new Pose2d(-40,-9, Math.toRadians(0)), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(40, -9, Math.toRadians(0)))
                                //pixel1

                                //ciclu1
                                //e micuta, e geloasa, tabla tiganeeeaascaaa

                                //ciclu2
////                                //parcare


                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}