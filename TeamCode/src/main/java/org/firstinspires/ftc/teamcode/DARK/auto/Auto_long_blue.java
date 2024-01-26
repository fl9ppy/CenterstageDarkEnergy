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

        TrajectorySequence pixel1 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-43, 15, Math.toRadians(0)))
                .waitSeconds(0.1)
                .build();

        TrajectorySequence pedrum = drive.trajectorySequenceBuilder(pixel1.end())
                .lineToLinearHeading(new Pose2d(-55, 7, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, 7, Math.toRadians(0)))
                .waitSeconds(0.1)
                .build();

        TrajectorySequence pixel2 = drive.trajectorySequenceBuilder(pedrum.end())
                .lineToLinearHeading(new Pose2d(47,35, Math.toRadians(0)))
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(54,35, Math.toRadians(0)))
                .build();

        TrajectorySequence parcare1 = drive.trajectorySequenceBuilder(pixel2.end())
                .lineToLinearHeading(new Pose2d(47,35, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, 7, Math.toRadians(0)))
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
            case LEFT: drive.followTrajectorySequence(pixel1);
                        sleep(200);
                        drive.followTrajectorySequence(pedrum);
                        sleep(200);
                        drive.followTrajectorySequence(pixel2);
                        sleep(200);
                        drive.followTrajectorySequence(parcare1);
                break;
            case CENTER: drive.followTrajectorySequence(pixel1);
                sleep(200);
                drive.followTrajectorySequence(pedrum);
                sleep(200);
                drive.followTrajectorySequence(pixel2);
                sleep(200);
                drive.followTrajectorySequence(parcare1);
                break;
            case RIGHT: drive.followTrajectorySequence(pixel1);
                sleep(200);
                drive.followTrajectorySequence(pedrum);
                sleep(200);
                drive.followTrajectorySequence(pixel2);
                sleep(200);
                drive.followTrajectorySequence(parcare1);
                break;
        }
        if (!opModeIsActive()) return;
    }
}