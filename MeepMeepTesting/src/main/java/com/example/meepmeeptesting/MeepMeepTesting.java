package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(90), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(-90)))
                                //preload
                                .lineToLinearHeading(new Pose2d(-40,30, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-31, 30, Math.toRadians(0)))
                                //pedrum
                                .lineToLinearHeading(new Pose2d(-40,30, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(-56,10,Math.toRadians(0)), Math.toRadians(-90))
                                .lineToSplineHeading(new Pose2d(38,10, Math.toRadians(0)))
                                //pixel1
                                .splineToSplineHeading(new Pose2d(50,47, Math.toRadians(0)), Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(45,45, Math.toRadians(0)))
                                //ciclu1
                                //e micuta, e geloasa, tabla tiganeeeaascaaa
                                .splineToSplineHeading(new Pose2d(37,10, Math.toRadians(0)), Math.toRadians(-90))
                                .lineToSplineHeading(new Pose2d(-58,10, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(38,10, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(50,47, Math.toRadians(0)), Math.toRadians(90))
                                //ciclu2
                                .lineToLinearHeading(new Pose2d(45,45, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(37,10, Math.toRadians(0)), Math.toRadians(-90))
                                .lineToSplineHeading(new Pose2d(-58,10, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(38,10, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(50,47, Math.toRadians(0)), Math.toRadians(90))
                                //parcare
                                .lineToLinearHeading(new Pose2d(45,45, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(58,10, Math.toRadians(0)), Math.toRadians(0))

                                .build());
        //auto long blue left

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}