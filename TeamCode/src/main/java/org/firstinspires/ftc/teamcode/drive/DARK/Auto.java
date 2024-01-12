package org.firstinspires.ftc.teamcode.drive.DARK;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DARK.DetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.DARK.RobotUtils;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Auto_long_red", group="AUTINOMOUSGOOD")
@Config


public class Auto extends LinearOpMode {
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
//        TRAIECTORII

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
                .lineToLinearHeading(new Pose2d(-44, -11, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-44,-35, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-44, -11, Math.toRadians(0)))
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

        TrajectorySequence preoutake = drive.trajectorySequenceBuilder(intake2.end())
                .lineToLinearHeading(new Pose2d(-55, -8, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30,Math.toRadians(180),DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(30))
                .lineToLinearHeading(new Pose2d(40, -5, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30,Math.toRadians(180),DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(30))
                .build();

        TrajectorySequence outake = drive.trajectorySequenceBuilder(preoutake.end())
                .lineToLinearHeading(new Pose2d(40,-35, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30,Math.toRadians(180),DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(30))
                .lineToLinearHeading(new Pose2d(45,-35, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30,Math.toRadians(180),DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(30))
                .build();

        TrajectorySequence preloadzona2 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-43, -15, Math.toRadians(0)))
                        .build();

        TrajectorySequence preloadzona1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35,-59, Math.toRadians(90)))
                        .build();

        TrajectorySequence preloadzona3 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-56, -28, Math.toRadians(0)), Math.toRadians(90))
                        .build();

        TrajectorySequence startleft = drive.trajectorySequenceBuilder(startPose)
                .forward(10)
                .turn(Math.toRadians(90))
                .forward(10)
                .build();


        while (!isStarted() && !isStopRequested()) {

            double zoneright = detectionPipeline.getZoneLuminosity(85);
            double zonemid = Math.min(Math.min(Math.min( detectionPipeline.getZoneLuminosity(34)
                                                        ,detectionPipeline.getZoneLuminosity(35))
                                                        ,detectionPipeline.getZoneLuminosity(24))
                                                        ,detectionPipeline.getZoneLuminosity(25));


            if (zoneright<zonemid && zoneright<115) zone = ZoneType.RIGHT;
            else if (zonemid < zoneright && zonemid<115)zone = ZoneType.CENTER;
            else zone = ZoneType.LEFT;


            telemetry.addData("zone = ",zone.toString());
            telemetry.addData("luminosity zone right",zoneright);
            telemetry.addData("luminosity zone mid",zonemid);
            loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
//            telemetry.addData("luminosity zone 6",detectionPipeline.getZoneLuminosity(8));
            telemetry.update();

            robot.portita_inchisa();
        }
        bCameraOpened = false;
        if(nu_stiu_sa_codez2) {
            zoneFinal = zone;
            nu_stiu_sa_codez2 = false;
        }
        switch(zoneFinal){
            case LEFT:
                drive.followTrajectorySequence(preloadzona1);
                sleep(450);
                drive.followTrajectorySequence(preoutake);
                sleep(450);
                robot.slider_up();
                sleep(300);
                robot.bratUp();
                sleep(450);
                drive.followTrajectorySequence(outake);
                sleep(300);
                robot.portita_deschis();
                sleep(300);
                robot.bratDown();
                sleep(300);
                robot.slider_down();
            case CENTER:
                drive.followTrajectorySequence(preloadzona2);
                sleep(450);
                drive.followTrajectorySequence(preoutake);
                sleep(450);
                robot.slider_up();
                sleep(300);
                robot.bratUp();
                sleep(450);
                drive.followTrajectorySequence(outake);
                sleep(300);
                robot.portita_deschis();
                sleep(300);
                robot.bratDown();
                sleep(300);
                robot.slider_down();
                break;
            case RIGHT:
                drive.followTrajectorySequence(preloadzona3);
                sleep(450);
                drive.followTrajectorySequence(preoutake);
                sleep(450);
                robot.slider_up();
                sleep(300);
                robot.bratUp();
                sleep(450);
                drive.followTrajectorySequence(outake);
                sleep(300);
                robot.portita_deschis();
                sleep(300);
                robot.bratDown();
                sleep(300);
                robot.slider_down();
        }
        if (!opModeIsActive()) return;
    }
}