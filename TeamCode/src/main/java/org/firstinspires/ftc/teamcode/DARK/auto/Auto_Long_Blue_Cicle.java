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

@Autonomous(name="Auto_Long_Blue_Cicle", group="AUTONOMOUSGOOD")
@Config


public class Auto_Long_Blue_Cicle extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(-35, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        //Tranjectories

//              ------------right----------
        TrajectorySequence pixel1r = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-40,30, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-31, 30, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-40,30, Math.toRadians(0)))
                .build();

        TrajectorySequence pedrumr = drive.trajectorySequenceBuilder(pixel1r.end())
                .lineToLinearHeading(new Pose2d(-47, 8, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, 8, Math.toRadians(0)))
                .build();

        TrajectorySequence pixel2r = drive.trajectorySequenceBuilder(pedrumr.end())
                .lineToLinearHeading(new Pose2d(47,41, Math.toRadians(0)))
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(55,41, Math.toRadians(0)))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence parcarer = drive.trajectorySequenceBuilder(pixel2r.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(47,41, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, 6, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60,6, Math.toRadians(0)))
                .build();


//              ------------center----------
        TrajectorySequence pixel1c = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-36,29, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(-36, 37, Math.toRadians(-90)))
                .build();

        TrajectorySequence pedrumc = drive.trajectorySequenceBuilder(pixel1c.end())
                .lineToLinearHeading(new Pose2d(-45, 37, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(-56,10,Math.toRadians(0)), Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(38,10, Math.toRadians(0)))
                .build();

        TrajectorySequence pixel2c = drive.trajectorySequenceBuilder(pedrumc.end())
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(50,36, Math.toRadians(0)), Math.toRadians(90))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence ciclu1drumc = drive.trajectorySequenceBuilder(pixel2c.end())
                .lineToLinearHeading(new Pose2d(45,36, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(37,10, Math.toRadians(0)), Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(-58,10, Math.toRadians(0)))
                .waitSeconds(0.2)
                .addTemporalMarker(0.4, ()->{robot.intake_power();})
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(38,10, Math.toRadians(0)))
                .build();

        TrajectorySequence ciclu1boardc = drive.trajectorySequenceBuilder(ciclu1drumc.end())
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(50,36, Math.toRadians(0)), Math.toRadians(90))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence ciclu2drumc = drive.trajectorySequenceBuilder(ciclu1boardc.end())
                .lineToLinearHeading(new Pose2d(45,36, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(37,10, Math.toRadians(0)), Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(-58,10, Math.toRadians(0)))
                .waitSeconds(0.2)
                .addTemporalMarker(0.4, ()->{robot.intake_power();})
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(38,10, Math.toRadians(0)))
                .build();

        TrajectorySequence ciclu2boardc = drive.trajectorySequenceBuilder(ciclu2drumc.end())
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(50,36, Math.toRadians(0)), Math.toRadians(90))
                .waitSeconds(0.200)
                .build();


        TrajectorySequence parcarec = drive.trajectorySequenceBuilder(ciclu2boardc.end())
                .lineToSplineHeading(new Pose2d(45,36, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(58,10, Math.toRadians(0)), Math.toRadians(0))
                .build();


//              ------------left----------
        TrajectorySequence pixel1l = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-40,30, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-31, 30, Math.toRadians(0)))
                .build();

        TrajectorySequence pedruml = drive.trajectorySequenceBuilder(pixel1l.end())
                .lineToLinearHeading(new Pose2d(-40,30, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-56,10,Math.toRadians(0)), Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(38,10, Math.toRadians(0)))
                .build();

        TrajectorySequence pixel2l = drive.trajectorySequenceBuilder(pedruml.end())
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(50,47, Math.toRadians(0)), Math.toRadians(90))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence ciclu1druml = drive.trajectorySequenceBuilder(pixel2l.end())
                .lineToLinearHeading(new Pose2d(45,45, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(37,10, Math.toRadians(0)), Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(-58,10, Math.toRadians(0)))
                .waitSeconds(0.2)
                .addTemporalMarker(0.4, ()->{robot.intake_power();})
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(38,10, Math.toRadians(0)))
                .build();

        TrajectorySequence ciclu1boardl = drive.trajectorySequenceBuilder(ciclu1druml.end())
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(50,47, Math.toRadians(0)), Math.toRadians(90))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence ciclu2druml = drive.trajectorySequenceBuilder(ciclu1boardl.end())
                .lineToLinearHeading(new Pose2d(45,45, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(37,10, Math.toRadians(0)), Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(-58,10, Math.toRadians(0)))
                .waitSeconds(0.2)
                .addTemporalMarker(0.4, ()->{robot.intake_power();})
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(38,10, Math.toRadians(0)))
                .build();

        TrajectorySequence ciclu2boardl = drive.trajectorySequenceBuilder(ciclu2druml.end())
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(50,47, Math.toRadians(0)), Math.toRadians(90))
                .waitSeconds(0.200)
                .build();


        TrajectorySequence parcarel = drive.trajectorySequenceBuilder(ciclu2boardl.end())
                .lineToLinearHeading(new Pose2d(45,45, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(58,10, Math.toRadians(0)), Math.toRadians(0))
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
        }

        bCameraOpened = false;

        if(nu_stiu_sa_codez2) {
            zoneFinal = zone;
            nu_stiu_sa_codez2 = false;
        }

        switch(zoneFinal){
            case LEFT:
                drive.followTrajectorySequence(pixel1l);
                sleep(4000);
                drive.followTrajectorySequence(pedruml);
                sleep(200);
                drive.followTrajectorySequence(pixel2l);
                sleep(200);
                robot.outake_open();
                sleep(500);
                drive.followTrajectorySequence(ciclu1druml);
                sleep(200);
                drive.followTrajectorySequence(ciclu1boardl);
                sleep(200);
                drive.followTrajectorySequence(ciclu2druml);
                sleep(200);
                drive.followTrajectorySequence(ciclu2boardl);
                sleep(200);
                drive.followTrajectorySequence(parcarel);
                break;
            case CENTER: drive.followTrajectorySequence(pixel1c);
                sleep(4000);
                drive.followTrajectorySequence(pedrumc);
                sleep(200);
                drive.followTrajectorySequence(pixel2c);
                sleep(200);
                robot.outake_open();
                sleep(500);
                drive.followTrajectorySequence(ciclu1drumc);
                sleep(200);
                drive.followTrajectorySequence(ciclu1boardc);
                sleep(200);
                drive.followTrajectorySequence(ciclu2drumc);
                sleep(200);
                drive.followTrajectorySequence(ciclu2boardc);
                sleep(200);
                drive.followTrajectorySequence(parcarec);
                break;
            case RIGHT:
                drive.followTrajectorySequence(pixel1l);
                sleep(4000);
                drive.followTrajectorySequence(pedruml);
                sleep(200);
                drive.followTrajectorySequence(pixel2l);
                sleep(200);
                robot.outake_open();
                sleep(500);
                drive.followTrajectorySequence(parcarel);
                break;
        }
        if (!opModeIsActive()) return;
    }
}