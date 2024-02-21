package org.firstinspires.ftc.teamcode.DARK.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DARK.detection.DetectionPipeline;
import org.firstinspires.ftc.teamcode.DARK.utils.RobotUtils;
import org.firstinspires.ftc.teamcode.DARK.utils.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Auto_Long_Red_Cicle", group="AUTONOMOUSGOOD")
@Config


public class Auto_Long_Red_Cicle extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        //Tranjectories

//              ------------right----------
        TrajectorySequence pixel1r = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-40,-30, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-30, -28, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-40,-30, Math.toRadians(0)))
                .build();

        TrajectorySequence pedrumr = drive.trajectorySequenceBuilder(pixel1r.end())
                .splineToLinearHeading(new Pose2d(-40,-9, Math.toRadians(0)), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(40, -9, Math.toRadians(0)))
                .build();

        TrajectorySequence pixel2r = drive.trajectorySequenceBuilder(pedrumr.end())
                .splineToSplineHeading(new Pose2d(45,-40, Math.toRadians(0)), Math.toRadians(-90))
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .lineToLinearHeading(new Pose2d(52, -40,Math.toRadians(0)))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence ciclu1drumr = drive.trajectorySequenceBuilder(pixel2r.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(45,-40, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(37,-9, Math.toRadians(0)), Math.toRadians(90))
                .build();

        TrajectorySequence ciclu1stackr = drive.trajectorySequenceBuilder(ciclu1drumr.end())
                .lineToSplineHeading(new Pose2d(-58, -11, Math.toRadians(0)))
                .addTemporalMarker(1, ()->{robot.intake_power();})
                .build();

        TrajectorySequence ciclu1boardr = drive.trajectorySequenceBuilder(ciclu1drumr.end())
                .lineToSplineHeading(new Pose2d(40, -9, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(45,-35, Math.toRadians(0)), Math.toRadians(-90))
                .addTemporalMarker(0.6, ()->{robot.slider_up();})
                .addTemporalMarker(0.8, ()->{robot.axonUp();})
                .lineToLinearHeading(new Pose2d(52, -35,Math.toRadians(0)))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence parcarer = drive.trajectorySequenceBuilder(ciclu1boardr.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(45,-35, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(58,-8, Math.toRadians(0)), Math.toRadians(0))
                .build();


//              ------------center----------
        TrajectorySequence pixel1c = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36, -27, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-36, -37, Math.toRadians(90)))
                .build();

        TrajectorySequence pedrumc = drive.trajectorySequenceBuilder(pixel1c.end())
                .lineToLinearHeading(new Pose2d(-45, -37, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(-54,-8,Math.toRadians(0)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(40, -9, Math.toRadians(0)))
                .build();

        TrajectorySequence pixel2c = drive.trajectorySequenceBuilder(pedrumc.end())
                .splineToSplineHeading(new Pose2d(45,-34, Math.toRadians(0)), Math.toRadians(-90))
                .addTemporalMarker(0.5, ()->{robot.slider_up();})
                .addTemporalMarker(0.7, ()->{robot.axonUp();})
                .lineToLinearHeading(new Pose2d(52, -34,Math.toRadians(0)))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence ciclu1drumc = drive.trajectorySequenceBuilder(pixel2c.end())
                .addTemporalMarker(0.6, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(45,-34, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(37,-9, Math.toRadians(0)), Math.toRadians(90))
                .build();

        TrajectorySequence ciclu1stackc = drive.trajectorySequenceBuilder(ciclu1drumc.end())
                .lineToSplineHeading(new Pose2d(-58, -11, Math.toRadians(0)))
                .addTemporalMarker(1, ()->{robot.intake_power();})
                .build();

        TrajectorySequence ciclu1boardc = drive.trajectorySequenceBuilder(ciclu1stackc.end())
                .lineToSplineHeading(new Pose2d(40, -9, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(45,-27, Math.toRadians(0)), Math.toRadians(-90))
                .addTemporalMarker(1.4, ()->{robot.slider_up();})
                .addTemporalMarker(1.6, ()->{robot.axonUp();})
                .splineToSplineHeading(new Pose2d(52,-42, Math.toRadians(0)), Math.toRadians(90))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence parcarec = drive.trajectorySequenceBuilder(ciclu1boardc.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToSplineHeading(new Pose2d(45,-42, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(58,-8, Math.toRadians(0)), Math.toRadians(0))
                .build();


//              ------------left----------
        TrajectorySequence pixel1l = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-40,-30, Math.toRadians(180)))
                .build();

        TrajectorySequence pedruml = drive.trajectorySequenceBuilder(pixel1l.end())
                .lineToLinearHeading(new Pose2d(-33,-30, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-33,-8, Math.toRadians(180)))
                .turn(Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(40, -9, Math.toRadians(0)))
                .build();

        TrajectorySequence pixel2l = drive.trajectorySequenceBuilder(pedruml.end())
                .splineToSplineHeading(new Pose2d(45,-27, Math.toRadians(0)), Math.toRadians(-90))
                .addTemporalMarker(0.5, ()->{robot.slider_up();})
                .addTemporalMarker(0.7, ()->{robot.axonUp();})
                .lineToLinearHeading(new Pose2d(52, -27,Math.toRadians(0)))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence ciclu1druml = drive.trajectorySequenceBuilder(pixel2l.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(45,-27, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(37,-9, Math.toRadians(0)), Math.toRadians(90))
                .build();

        TrajectorySequence ciclu1stackl = drive.trajectorySequenceBuilder(ciclu1druml.end())
                .lineToSplineHeading(new Pose2d(-58,11, Math.toRadians(0)))
                .addTemporalMarker(1, ()->{robot.intake_power();})
                .build();

        TrajectorySequence ciclu1boardl = drive.trajectorySequenceBuilder(ciclu1druml.end())
                .lineToSplineHeading(new Pose2d(40, -9, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(45,-34, Math.toRadians(0)), Math.toRadians(-90))
                .addTemporalMarker(0.9, ()->{robot.slider_up();})
                .addTemporalMarker(0.11, ()->{robot.axonUp();})
                .lineToLinearHeading(new Pose2d(52, -34,Math.toRadians(0)))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence parcarel = drive.trajectorySequenceBuilder(ciclu1boardl.end())
                .lineToLinearHeading(new Pose2d(45,-34, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(58,-8, Math.toRadians(0)), Math.toRadians(0))
                .build();




        while (!isStarted() && !isStopRequested()) {

            double zoneright = detectionPipeline.getZoneLuminosity(85);
            double zonemid = Math.min(Math.min(Math.min( detectionPipeline.getZoneLuminosity(34)
                                    ,detectionPipeline.getZoneLuminosity(35))
                            ,detectionPipeline.getZoneLuminosity(24))
                    ,detectionPipeline.getZoneLuminosity(25));

            if (zoneright<zonemid && zoneright<80) zone = ZoneType.RIGHT;
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

            robot.outake_close();
            robot.extension_up();
        }

        bCameraOpened = false;

        if(nu_stiu_sa_codez2) {
            zoneFinal = zone;
            nu_stiu_sa_codez2 = false;
        }

        switch(zoneFinal){
            case LEFT:
                drive.followTrajectorySequence(pixel1l);
                sleep(100);
                drive.followTrajectorySequence(pedruml);
                sleep(100);
                drive.followTrajectorySequence(pixel2l);
                sleep(200);
                robot.outake_open();
                sleep(100);
                drive.followTrajectorySequence(ciclu1druml);
                sleep(100);
                drive.followTrajectorySequence(ciclu1stackl);
                sleep(1000);
                robot.startIntakeAuto(1, 0.05, 100, 0.01);
                robot.stopIntakeAuto(1000);
                robot.outake_close();
                drive.followTrajectorySequence(ciclu1boardl);
                robot.outake_open();
                sleep(200);
                drive.followTrajectorySequence(parcarel);
                break;
            case CENTER: drive.followTrajectorySequence(pixel1c);
                sleep(100);
                drive.followTrajectorySequence(pedrumc);
                sleep(100);
                drive.followTrajectorySequence(pixel2c);
                sleep(200);
                robot.outake_open();
                sleep(100);
                drive.followTrajectorySequence(ciclu1drumc);
                sleep(100);
                drive.followTrajectorySequence(ciclu1stackc);
                sleep(1000);
                robot.startIntakeAuto(1, 0.05, 100, 0.01);
                robot.stopIntakeAuto(1000);
                robot.outake_close();
                sleep(5000);
                drive.followTrajectorySequence(ciclu1boardc);
                robot.outake_open();
                sleep(200);
                drive.followTrajectorySequence(parcarec);
                break;
            case RIGHT:
                drive.followTrajectorySequence(pixel1r);
                sleep(100);
                drive.followTrajectorySequence(pedrumr);
                sleep(100);
                drive.followTrajectorySequence(pixel2r);
                sleep(200);
                robot.outake_open();
                sleep(100);
                drive.followTrajectorySequence(ciclu1drumr);
                sleep(100);
                drive.followTrajectorySequence(ciclu1stackr);
                sleep(1000);
                robot.startIntakeAuto(1, 0.05, 100, 0.01);
                robot.stopIntakeAuto(1000);
                robot.outake_close();
                sleep(5000);
                drive.followTrajectorySequence(ciclu1boardr);
                robot.outake_open();
                sleep(100);
                drive.followTrajectorySequence(parcarer);
                break;
        }
        if (!opModeIsActive()) return;
    }
}