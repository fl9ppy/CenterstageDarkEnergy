package org.firstinspires.ftc.teamcode.drive.DARK;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="test_tunning", group="Linear Opmode")
@Config

public class test extends LinearOpMode {
    private SampleMecanumDrive drive;
    private RobotUtils robot;
    @Override
    public void runOpMode() {

        robot = new RobotUtils(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


//        TrajectorySequence startleft = drive.trajectorySequenceBuilder(startPose)
//                .forward(30)
              //  .build();
        TrajectorySequence preload= drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-44, -24, Math.toRadians(0)))
                .build();
//
        TrajectorySequence intake1= drive.trajectorySequenceBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(-62,-11, Math.toRadians(0)))
                .build();

        TrajectorySequence outtake1= drive.trajectorySequenceBuilder(intake1.end())
                .lineToLinearHeading(new Pose2d(44, -11, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(44,-35, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(44, -11, Math.toRadians(0)))
                .addTemporalMarker(0.20, ()-> {robot.slider_up();
                        robot.bratUp();
                    })
                .build();

        TrajectorySequence intake2 = drive.trajectorySequenceBuilder(outtake1.end())
                .lineToLinearHeading(new Pose2d(-62,-11, Math.toRadians(0)))
                .addTemporalMarker(0.5, ()->{robot.bratDown();
                    robot.slider_down();
                    })
                .build();

        TrajectorySequence outake2 = drive.trajectorySequenceBuilder(intake2.end())
                .lineToLinearHeading(new Pose2d(44, -11, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(44,-35, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(44, -11, Math.toRadians(0)))
                .addTemporalMarker(0.5, ()-> {robot.slider_up();
                    robot.bratUp();
                    })
                .addTemporalMarker(0.5, ()->{})
                .build();

//        TrajectorySequence preloadzona1 = drive.trajectorySequenceBuilder(startPose)
//                .lineToSplineHeading(new Pose2d(-34, -29, Math.toRadians(0)))
//                        .build();
//
//        TrajectorySequence preloadzona2 = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-35,-59, Math.toRadians(90)))
//                        .build();
//
//        TrajectorySequence preloadzona3 = drive.trajectorySequenceBuilder(startPose)
//                .splineToLinearHeading(new Pose2d(-56, -28, Math.toRadians(0)), Math.toRadians(90))
//                        .build();

        waitForStart();


    drive.followTrajectorySequence(preload);
    sleep(100);
  //  drive.followTrajectorySequence(startleft);
    drive.followTrajectorySequence(intake1);
    sleep(100);
    robot.portita_deschis();
    sleep(100);
    robot.intakerotire();
    if(robot.hasDetected())
        robot.intakeoprire();
    sleep(100);
    robot.portita_inchisa();
    sleep(100);
    drive.followTrajectorySequence(outake2);
    sleep(100);
    robot.portita_deschis();
    sleep(100);
    drive.followTrajectorySequence(intake1);
    sleep(100);
    robot.portita_deschis();
    sleep(100);
    robot.intakerotire();
    if(robot.hasDetected())
        robot.intakeoprire();
    sleep(100);
    robot.portita_inchisa();
    sleep(100);
    drive.followTrajectorySequence(outake2);
    sleep(100);
    robot.portita_deschis();
    sleep(100);
    }
}
