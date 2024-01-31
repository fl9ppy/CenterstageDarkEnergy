package org.firstinspires.ftc.teamcode.DARK.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DARK.detection.DetectionPipeline;
import org.firstinspires.ftc.teamcode.DARK.utils.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.DARK.utils.RobotUtils;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Auto_long_blue", group="AUTONOMOUSGOOD")
@Config


public class Auto_long_blue extends LinearOpMode {
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
                .lineToLinearHeading(new Pose2d(-54,29, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-60,29, Math.toRadians(0)))
                .build();

        TrajectorySequence pedrumr = drive.trajectorySequenceBuilder(pixel1r.end())
                .lineToLinearHeading(new Pose2d(-61, 7, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(45, 7, Math.toRadians(0)))
                .build();

        TrajectorySequence pixel2r = drive.trajectorySequenceBuilder(pedrumr.end())
                .lineToLinearHeading(new Pose2d(45,29, Math.toRadians(0)))
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(55,29, Math.toRadians(0)))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence parcarer = drive.trajectorySequenceBuilder(pixel2r.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(45,29, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, 8, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60,8, Math.toRadians(0)))
                .build();


//              ------------center----------
        TrajectorySequence pixel1c = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-43, 29, Math.toRadians(0)))
                .build();

        TrajectorySequence pedrumc = drive.trajectorySequenceBuilder(pixel1c.end())
                .lineToLinearHeading(new Pose2d(-50, 5, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, 5, Math.toRadians(0)))
                .build();

        TrajectorySequence pixel2c = drive.trajectorySequenceBuilder(pedrumc.end())
                .lineToLinearHeading(new Pose2d(47,35, Math.toRadians(0)))
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(53,35, Math.toRadians(0)))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence parcarec = drive.trajectorySequenceBuilder(pixel2c.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(47,35, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, 6, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60,6, Math.toRadians(0)))
                .build();


//              ------------left----------
        TrajectorySequence pixel1l = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-40,30, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-26, 30, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-40,30, Math.toRadians(0)))
                .build();

        TrajectorySequence pedruml = drive.trajectorySequenceBuilder(pixel1l.end())
                .lineToLinearHeading(new Pose2d(-47, 8, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, 8, Math.toRadians(0)))
                .build();

        TrajectorySequence pixel2l = drive.trajectorySequenceBuilder(pedruml.end())
                .lineToLinearHeading(new Pose2d(47,41, Math.toRadians(0)))
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(55,41, Math.toRadians(0)))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence parcarel = drive.trajectorySequenceBuilder(pixel2l.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(47,41, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, 6, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60,6, Math.toRadians(0)))
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
                        sleep(200);
                        drive.followTrajectorySequence(pedruml);
                        sleep(200);
                        drive.followTrajectorySequence(pixel2l);
                        sleep(200);
                        robot.outake_open();
                        sleep(500);
                        drive.followTrajectorySequence(parcarel);
                break;
            case CENTER: drive.followTrajectorySequence(pixel1c);
                sleep(200);
                drive.followTrajectorySequence(pedrumc);
                sleep(200);
                drive.followTrajectorySequence(pixel2c);
                sleep(200);
                robot.outake_open();
                sleep(500);
                drive.followTrajectorySequence(parcarec);
                break;
            case RIGHT: drive.followTrajectorySequence(pixel1r);
                sleep(200);
                drive.followTrajectorySequence(pedrumr);
                sleep(200);
                drive.followTrajectorySequence(pixel2r);
                sleep(200);
                robot.outake_open();
                sleep(500);
                drive.followTrajectorySequence(parcarer);
                break;
        }
        if (!opModeIsActive()) return;
    }
}