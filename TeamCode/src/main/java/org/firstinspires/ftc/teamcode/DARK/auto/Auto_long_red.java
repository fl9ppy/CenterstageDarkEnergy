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

@Autonomous(name="Auto_long_red", group="AUTONOMOUSGOOD")
@Config


public class Auto_long_red extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Tranjectories

//              ------------right----------
        TrajectorySequence pixel1r = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-40,-30, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-30, -28, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-40,-30, Math.toRadians(0)))
                .build();

        TrajectorySequence pedrumr = drive.trajectorySequenceBuilder(pixel1r.end())
                .lineToLinearHeading(new Pose2d(-47, -8, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, -8, Math.toRadians(0)))
                .build();

        TrajectorySequence pixel2r = drive.trajectorySequenceBuilder(pedrumr.end())
                .lineToLinearHeading(new Pose2d(47,-39, Math.toRadians(0)))
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(55,-39, Math.toRadians(0)))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence parcarer = drive.trajectorySequenceBuilder(pixel2r.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(47,-41, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, -7, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(59,-7, Math.toRadians(0)))
                .build();

//              ------------center----------
        TrajectorySequence pixel1c = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-42, -16, Math.toRadians(0)))
                .build();

        TrajectorySequence pedrumc = drive.trajectorySequenceBuilder(pixel1c.end())
                .lineToLinearHeading(new Pose2d(-56, -15, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-56,-8,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, -8, Math.toRadians(0)))
                .build();

        TrajectorySequence pixel2c = drive.trajectorySequenceBuilder(pedrumc.end())
                .lineToLinearHeading(new Pose2d(47,-34, Math.toRadians(0)))
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(54,-34, Math.toRadians(0)))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence parcarec = drive.trajectorySequenceBuilder(pixel2c.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(47,-33, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, -6, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(58,-6, Math.toRadians(0)))
                .build();


//              ------------left----------
        TrajectorySequence pixel1l = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-38,-29, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-34,-26, Math.toRadians(180)))
                .build();

        TrajectorySequence pedruml = drive.trajectorySequenceBuilder(pixel1l.end())
                .lineToLinearHeading(new Pose2d(-35,-6, Math.toRadians(180)))
                .turn(Math.toRadians(-180))
                .lineToLinearHeading(new Pose2d(47, -6, Math.toRadians(0)))
                .build();

        TrajectorySequence pixel2l = drive.trajectorySequenceBuilder(pedruml.end())
                .lineToLinearHeading(new Pose2d(47,-41, Math.toRadians(0)))
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .waitSeconds(0.300)
                .lineToLinearHeading(new Pose2d(57,-39, Math.toRadians(0)))
                .waitSeconds(0.200)
                .build();

        TrajectorySequence parcarel = drive.trajectorySequenceBuilder(pixel2l.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(47,-41, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, -8, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60,-8, Math.toRadians(0)))
                .build();



        while (!isStarted() && !isStopRequested()) {

            double zoneright = detectionPipeline.getZoneLuminosity(85);
            double zonemid = Math.min(Math.min(Math.min( detectionPipeline.getZoneLuminosity(34)
                                    ,detectionPipeline.getZoneLuminosity(35))
                            ,detectionPipeline.getZoneLuminosity(24))
                    ,detectionPipeline.getZoneLuminosity(25));

            if (zoneright<zonemid && zoneright<140) zone = ZoneType.RIGHT;
            else if (zonemid < zoneright && zonemid<90)zone = ZoneType.CENTER;
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
                sleep(400);
                robot.outake_open();
                sleep(500);
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
                drive.followTrajectorySequence(parcarec);
                break;
            case RIGHT: drive.followTrajectorySequence(pixel1r);
                sleep(4000);
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