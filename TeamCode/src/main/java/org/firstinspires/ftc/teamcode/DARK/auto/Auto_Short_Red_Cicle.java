package org.firstinspires.ftc.teamcode.DARK.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DARK.detection.DetectionPipeline;
import org.firstinspires.ftc.teamcode.DARK.utils.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.DARK.utils.RobotUtils;
import org.firstinspires.ftc.teamcode.drive.opmode.StrafeTest;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Auto_Short_Red_Cicle", group="AUTONOMOUSGOOD")
@Config


public class Auto_Short_Red_Cicle extends LinearOpMode {
    private RobotUtils robot;
    OpenCvCamera webcam;
    DetectionPipeline detectionPipeline;

    boolean bCameraOpened = false;
    private SampleMecanumDrive drive;
    private double loopTime=0,loop;

    private  boolean nu_stiu_sa_codez2 = true;

    enum ZoneType{
        RIGHT,
        LEFT,
        CENTER
    }
    ZoneType zone =  ZoneType.CENTER;
    ZoneType zoneFinal =  ZoneType.CENTER;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detectionPipeline = new DetectionPipeline();
        webcam.setPipeline(detectionPipeline);
        detectionPipeline.setGridSize(10);
        drive = new SampleMecanumDrive(hardwareMap);
        robot = new RobotUtils(hardwareMap);
        sleep(2000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                bCameraOpened = true;
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        Pose2d startPose = new Pose2d(10, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        //Tranjectories

//              ------------right----------
        TrajectorySequence pixel1r = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(13, -33, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(4,-33,Math.toRadians(180)))
                .turn(Math.toRadians(60))
                .build();

        TrajectorySequence pedrumr = drive.trajectorySequenceBuilder(pixel1r.end())
                .lineToSplineHeading(new Pose2d(15, -33, Math.toRadians(180)))
                .turn(Math.toRadians(180))
                .build();

        TrajectorySequence pixel2r = drive.trajectorySequenceBuilder(pedrumr.end())
                .addTemporalMarker(0.2, ()->{robot.slider_up_preload();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .splineToLinearHeading(new Pose2d(50,-25, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence ciclu1drumr = drive.trajectorySequenceBuilder(pixel2r.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToSplineHeading(new Pose2d(45,-25, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(37,-56, Math.toRadians(0)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-36,-56, Math.toRadians(0)))
                .build();

        TrajectorySequence ciclu1stackr = drive.trajectorySequenceBuilder(ciclu1drumr.end())
                .lineToLinearHeading(new Pose2d(-61, -31, Math.toRadians(0)))
                .build();

        TrajectorySequence ciclu1druminapoir = drive.trajectorySequenceBuilder(ciclu1stackr.end())
                .lineToSplineHeading(new Pose2d(-56,-31, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-42,-57, Math.toRadians(0)), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(37,-56, Math.toRadians(0)))
                .build();

        TrajectorySequence ciclu1boardr = drive.trajectorySequenceBuilder(ciclu1druminapoir.end())
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.4, ()->{robot.axonUp();})
                .splineToSplineHeading(new Pose2d(52,-31, Math.toRadians(0)), Math.toRadians(-90))
                .build();

        TrajectorySequence parcarer = drive.trajectorySequenceBuilder(ciclu1boardr.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(45,-31, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(50,-58, Math.toRadians(0)), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(58,-58, Math.toRadians(0)))
                .build();


        ///       ------------center----------
        TrajectorySequence pixel1c = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(10, -28, Math.toRadians(-90)))
                .build();

        TrajectorySequence pedrumc = drive.trajectorySequenceBuilder(pixel1c.end())
                .lineToLinearHeading(new Pose2d(10,-38, Math.toRadians(-90)))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence pixel2c = drive.trajectorySequenceBuilder(pedrumc.end())
                .addTemporalMarker(0.2, ()->{robot.slider_up_preload();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .lineToLinearHeading(new Pose2d(51,-34, Math.toRadians(0)))
                .build();

        TrajectorySequence ciclu1drumc = drive.trajectorySequenceBuilder(pixel2r.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(45,-26, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(37,-56, Math.toRadians(0)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-36,-56, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-47, -34, Math.toRadians(0)))
                .build();

        TrajectorySequence ciclu1stackc = drive.trajectorySequenceBuilder(ciclu1drumc.end())
                .lineToLinearHeading(new Pose2d(-61, -31, Math.toRadians(0)))
                .build();

        TrajectorySequence ciclu1druminapoic = drive.trajectorySequenceBuilder(ciclu1stackc.end())
                .lineToLinearHeading(new Pose2d(-55,-31, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-42,-56, Math.toRadians(0)), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(37,-56, Math.toRadians(0)))
                .build();

        TrajectorySequence ciclu1boardc = drive.trajectorySequenceBuilder(ciclu1druminapoic.end())
                .splineToSplineHeading(new Pose2d(45,-40, Math.toRadians(0)), Math.toRadians(-90))
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.4, ()->{robot.axonUp();})
                .lineToLinearHeading(new Pose2d(52, -37,Math.toRadians(0)))
                .build();

        TrajectorySequence parcarec = drive.trajectorySequenceBuilder(ciclu1boardr.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(45,-40, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(50,-58, Math.toRadians(0)), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(58,-58, Math.toRadians(0)))
                .build();

        ///     ------------left----------
        TrajectorySequence pixel1l = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(10, -36, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(25,-36, Math.toRadians(-90)))
                .build();

        TrajectorySequence pedruml = drive.trajectorySequenceBuilder(pixel1l.end())
                .lineToLinearHeading(new Pose2d(25,-49, Math.toRadians(-90)))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence pixel2l = drive.trajectorySequenceBuilder(pedruml.end())
                .addTemporalMarker(0.2, ()->{robot.slider_up_preload();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .lineToLinearHeading(new Pose2d(52,-37, Math.toRadians(0)))
                .waitSeconds(0.200)
                .build();


        TrajectorySequence ciclu1druml = drive.trajectorySequenceBuilder(pixel2l.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(45,-26, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(37,-56, Math.toRadians(0)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-36,-56, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-47, -34, Math.toRadians(0)))
                .build();

        TrajectorySequence ciclu1stackl = drive.trajectorySequenceBuilder(ciclu1druml.end())
                .lineToLinearHeading(new Pose2d(-61, -31, Math.toRadians(0)))
                .build();

        TrajectorySequence ciclu1druminapoil = drive.trajectorySequenceBuilder(ciclu1stackl.end())
                .lineToLinearHeading(new Pose2d(-55,-31, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-42,-57, Math.toRadians(0)), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(37,-57, Math.toRadians(0)))
                .build();

        TrajectorySequence ciclu1boardl = drive.trajectorySequenceBuilder(ciclu1druminapoil.end())
                .splineToSplineHeading(new Pose2d(45,-31, Math.toRadians(0)), Math.toRadians(-90))
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.4, ()->{robot.axonUp();})
                .lineToLinearHeading(new Pose2d(52, -31,Math.toRadians(0)))
                .build();

        TrajectorySequence parcarel = drive.trajectorySequenceBuilder(ciclu1boardl.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(45,-31, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(50,-58, Math.toRadians(0)), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(58,-58, Math.toRadians(0)))
                .build();


        while (!isStarted() && !isStopRequested()) {

            double zoneright = detectionPipeline.getZoneLuminosity(85);
            double zonemid = Math.min(Math.min(Math.min( detectionPipeline.getZoneLuminosity(34)
                                    ,detectionPipeline.getZoneLuminosity(35))
                            ,detectionPipeline.getZoneLuminosity(24))
                    ,detectionPipeline.getZoneLuminosity(25));

            if (zoneright<zonemid && zoneright<112) zone = ZoneType.RIGHT;
            else if (zonemid < zoneright && zonemid<80)zone = ZoneType.CENTER;
            else zone = ZoneType.LEFT;

            telemetry.addData("zone = ",zone.toString());
            telemetry.addData("luminosity zone right",zoneright);
            telemetry.addData("luminosity zone mid",zonemid);
            telemetry.addData("slider1", robot.slider1.getCurrentPosition());
            telemetry.addData("slider2", robot.slider2.getCurrentPosition());

            loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;

            telemetry.update();

            robot.extension_up();
            robot.outake_close();
        }

        bCameraOpened = false;

        if(nu_stiu_sa_codez2) {
            zoneFinal = zone;
            nu_stiu_sa_codez2 = false;
        }

        switch(zoneFinal){
            case LEFT:
                drive.followTrajectorySequence(pixel1r);
                drive.followTrajectorySequence(pedrumr);
                drive.followTrajectorySequence(pixel2r);
                robot.outake_open();
                drive.followTrajectorySequence(ciclu1drumr);
                drive.followTrajectorySequence(ciclu1stackr);
                robot.stack1();
                robot.inverse_intake_power();
                sleep(2500);
                robot.outake_close();
                robot.stopIntake();
                robot.extension_up();
                drive.followTrajectorySequence(ciclu1druminapoir);
                drive.followTrajectorySequence(ciclu1boardr);
                robot.outake_open();
                drive.followTrajectorySequence(parcarer);
                break;
            case CENTER:
                drive.followTrajectorySequence(pixel1c);
                sleep(50);
                drive.followTrajectorySequence(pedrumc);
                drive.followTrajectorySequence(pixel2c);
                sleep(50);
                robot.outake_open();
                sleep(100);
                drive.followTrajectorySequence(ciclu1drumc);
                drive.followTrajectorySequence(ciclu1stackc);
                sleep(50);
                robot.stack1();
                robot.inverse_intake_power();
                sleep(3000);
                robot.outake_close();
                robot.stopIntake();
                robot.extension_up();
                drive.followTrajectorySequence(ciclu1druminapoic);
                drive.followTrajectorySequence(ciclu1boardc);
                robot.outake_open();
                sleep(100);
                drive.followTrajectorySequence(parcarec);
                break;
            case RIGHT:
            drive.followTrajectorySequence(pixel1l);
            sleep(50);
            drive.followTrajectorySequence(pedruml);
            sleep(50);
            drive.followTrajectorySequence(pixel2l);
            sleep(100);
            robot.outake_open();
            sleep(50);
            drive.followTrajectorySequence(ciclu1druml);
            drive.followTrajectorySequence(ciclu1stackl);
            sleep(100);
            robot.stack1();
            robot.inverse_intake_power();
            sleep(3000);
            robot.outake_close();
            robot.stopIntake();
            robot.extension_up();
            drive.followTrajectorySequence(ciclu1druminapoil);
            drive.followTrajectorySequence(ciclu1boardl);
            robot.outake_open();
            sleep(100);
            drive.followTrajectorySequence(parcarel);
            break;
        }
        if (!opModeIsActive()) return;
    }
}