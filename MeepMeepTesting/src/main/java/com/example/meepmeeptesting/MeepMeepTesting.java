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
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-38,-29, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-29,-26, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-35,-9, Math.toRadians(180)))
                                .turn(Math.toRadians(-180))
                                .lineToLinearHeading(new Pose2d(47, -9, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(47,-41, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(55,-41, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(47,-41, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(47, -8, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(60,-8, Math.toRadians(0)))
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}