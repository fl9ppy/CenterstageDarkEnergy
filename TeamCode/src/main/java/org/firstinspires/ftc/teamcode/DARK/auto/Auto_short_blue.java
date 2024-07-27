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

@Autonomous(name="Auto_short_blue", group="AUTONOMOUSGOOD")
@Config


public class Auto_short_blue extends LinearOpMode {
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

        //Tranjectories miawwwwwwwwww

//              ------------right----------
        TrajectorySequence pixel1r = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(13, 33, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(4,33,Math.toRadians(180)))
                .turn(Math.toRadians(60))
                .build();

        TrajectorySequence pedrumr = drive.trajectorySequenceBuilder(pixel1r.end())
                .lineToLinearHeading(new Pose2d(15, 33, Math.toRadians(180)))
                .turn(Math.toRadians(180))
                .build();

        TrajectorySequence pixel2r = drive.trajectorySequenceBuilder(pedrumr.end())
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .lineToLinearHeading(new Pose2d(50,25, Math.toRadians(0)))
                .build();

        TrajectorySequence parcarer = drive.trajectorySequenceBuilder(pixel2r.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(45,29, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(45,59, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(56,59, Math.toRadians(0)))
                .build();


//              ------------center----------
        TrajectorySequence pixel1c = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(10, 28, Math.toRadians(-90)))
                .build();

        TrajectorySequence pedrumc = drive.trajectorySequenceBuilder(pixel1c.end())
                .lineToLinearHeading(new Pose2d(10,38, Math.toRadians(-90)))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence pixel2c = drive.trajectorySequenceBuilder(pedrumc.end())
                .addTemporalMarker(0.2, ()->{robot.slider_up();})
                .addTemporalMarker(0.3, ()->{robot.axonUp();})
                .lineToLinearHeading(new Pose2d(51,34, Math.toRadians(0)))
                .build();

        TrajectorySequence parcarec = drive.trajectorySequenceBuilder(pixel2c.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(45,36, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(45,57, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(58,59, Math.toRadians(0)))
                .build();


//              ------------left----------
        TrajectorySequence pixel1l = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(10, 36, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(25,36, Math.toRadians(-90)))
                .build();

        TrajectorySequence pedruml = drive.trajectorySequenceBuilder(pixel1l.end())
                .lineToLinearHeading(new Pose2d(25,49, Math.toRadians(-90)))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence pixel2l = drive.trajectorySequenceBuilder(pedruml.end())
                .addTemporalMarker(0.3, ()->{robot.slider_up();})
                .addTemporalMarker(0.4, ()->{robot.axonUp();})
                .lineToLinearHeading(new Pose2d(52,36, Math.toRadians(0)))
                .build();

        TrajectorySequence parcarel = drive.trajectorySequenceBuilder(pixel2l.end())
                .addTemporalMarker(0.5, ()->{robot.axonDown(); robot.slider_down();})
                .lineToLinearHeading(new Pose2d(45,29, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(45,59, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(59,57, Math.toRadians(0)))
                .build();




        while (!isStarted() && !isStopRequested()) {

            double zoneright = Math.min(Math.min(Math.min( detectionPipeline.getZoneLuminosity(86)
                                    ,detectionPipeline.getZoneLuminosity(86))
                            ,detectionPipeline.getZoneLuminosity(87))
                    ,detectionPipeline.getZoneLuminosity(87));
            double zonemid = Math.min(Math.min(Math.min( detectionPipeline.getZoneLuminosity(26)
                                    ,detectionPipeline.getZoneLuminosity(26))
                            ,detectionPipeline.getZoneLuminosity(27))
                    ,detectionPipeline.getZoneLuminosity(27));

            if(Math.abs(zoneright - zonemid) > 30){
                if(zonemid<zoneright) zone =ZoneType.CENTER;
                else if(zoneright<zonemid) zone = ZoneType.RIGHT;
            }
            else zone =ZoneType.LEFT;

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
                sleep(50);
                drive.followTrajectorySequence(pedruml);
                sleep(50);
                drive.followTrajectorySequence(pixel2l);
                sleep(50);
                robot.outake_open();
                sleep(100);
                drive.followTrajectorySequence(parcarel);
                break;
            case CENTER: drive.followTrajectorySequence(pixel1c);
                sleep(50);
                drive.followTrajectorySequence(pedrumc);
                sleep(50);
                drive.followTrajectorySequence(pixel2c);
                sleep(50);
                robot.outake_open();
                sleep(100);
                drive.followTrajectorySequence(parcarec);
                break;
            case RIGHT: drive.followTrajectorySequence(pixel1r);
                sleep(50);
                drive.followTrajectorySequence(pedrumr);
                sleep(50);
                drive.followTrajectorySequence(pixel2r);
                sleep(50);
                robot.outake_open();
                sleep(100);
                drive.followTrajectorySequence(parcarer);
                break;
        }
        if (!opModeIsActive()) return;
    }
}