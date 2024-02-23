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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(90), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                                //preload
                                .lineToLinearHeading(new Pose2d(-36, -27, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-36, -37, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-45, -37, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(-54,-8,Math.toRadians(0)), Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d(40, -9, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(45,-34, Math.toRadians(0)), Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(52, -34,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(45,-34, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(37,-4, Math.toRadians(0)), Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d(-58, -4, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(40, -9, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(52,-42, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(45,-42, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(58,-8, Math.toRadians(0)), Math.toRadians(0))
//                                .lineToLinearHeading(new Pose2d(-40,-30, Math.toRadians(180)))
//                                //pedrum
//                                .lineToLinearHeading(new Pose2d(-33,-30, Math.toRadians(180)))
//                                .lineToLinearHeading(new Pose2d(-33,-8, Math.toRadians(180)))
//                                .turn(Math.toRadians(180))
//                                .lineToSplineHeading(new Pose2d(40, -9, Math.toRadians(0)))
//                                //pixel1
//                                .splineToSplineHeading(new Pose2d(45,-27, Math.toRadians(0)), Math.toRadians(-90))
//                                .lineToLinearHeading(new Pose2d(52, -27,Math.toRadians(0)))
//                                //ciclu1
//                                .lineToLinearHeading(new Pose2d(45,-27, Math.toRadians(0)))
//                                .splineToSplineHeading(new Pose2d(37,-9, Math.toRadians(0)), Math.toRadians(90))
//                                .lineToSplineHeading(new Pose2d(-58, -11, Math.toRadians(0)))
//                                .lineToSplineHeading(new Pose2d(40, -9, Math.toRadians(0)))
//                                .splineToSplineHeading(new Pose2d(45,-34, Math.toRadians(0)), Math.toRadians(-90))
//                                .lineToLinearHeading(new Pose2d(52, -34,Math.toRadians(0)))


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