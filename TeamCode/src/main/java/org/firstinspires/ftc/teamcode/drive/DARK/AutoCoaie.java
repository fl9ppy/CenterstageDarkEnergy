///*
// * Copyright (c) 2021 OpenFTC Team
// *
// * Permission is hereby granted, free of charge, to any person obtaining a copy
// * of this software and associated documentation files (the "Software"), to deal
// * in the Software without restriction, including without limitation the rights
// * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// * copies of the Software, and to permit persons to whom the Software is
// * furnished to do so, subject to the following conditions:
// *
// * The above copyright notice and this permission notice shall be included in all
// * copies or substantial portions of the Software.
// * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// * SOFTWARE.
// */
//
//package org.firstinspires.ftc.teamcode.drive.DARK;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
//import com.outoftheboxrobotics.photoncore.PhotonLynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//
//@Autonomous(name="Auto_right_mid", group="Linear Opmode")
//@Config
//
//public class AutoCoaie extends LinearOpMode
//{
//    //INTRODUCE VARIABLES HERE
//
//    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
//    private SampleMecanumDrive drive;
//    private RobotUtils robot;
//
//
//    static final double FEET_PER_METER = 3.28084;
//
//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C920 webcam at 800x448.
//    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//
//    // UNITS ARE METERS
//    double tagsize = 0.166;
//
//    // Tag ID 1,2,3 from the 36h11 family
//    /*EDIT IF NEEDED!!!*/
//
//    int LEFT = 1;
//    int MIDDLE = 2;
//    int RIGHT = 3;
//
//    AprilTagDetection tagOfInterest = null;
//
//    @Override
//    public void runOpMode()
//    {
//        telemetry.setMsTransmissionInterval(50);
//
//        robot = new RobotUtils(hardwareMap);
//
//        drive = new SampleMecanumDrive(hardwareMap);
//        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        robot.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        Pose2d startPose = new Pose2d(35,-60,Math.toRadians(90));
//        drive.setPoseEstimate(startPose);
//
//        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)
//                .addTemporalMarker(0.5, ()->{robot.goHigh();})
//                .addTemporalMarker(1, ()->{robot.tureta_s();})
//                .lineToSplineHeading(new Pose2d(10, -60, Math.toRadians(90)))
//                .lineToSplineHeading(new Pose2d(10, -23, Math.toRadians(90)))
//                .build();
//
//        TrajectorySequence preintake = drive.trajectorySequenceBuilder(preload.end())
//                .lineToSplineHeading(new Pose2d(10, -12, Math.toRadians(0)))
//                .addTemporalMarker(0.5, ()->{robot.con1();})
//                .addTemporalMarker(0.7, ()->{robot.tureta_n();})
//                .build();
//
//        TrajectorySequence intake1 = drive.trajectorySequenceBuilder(preintake.end())
//                .lineToLinearHeading(new Pose2d(59, -8, Math.toRadians(0)))
//                .build();
//
//        TrajectorySequence outake1 = drive.trajectorySequenceBuilder(intake1.end())
//                .addTemporalMarker(0.01, ()->{robot.goMedium();})
//                .addTemporalMarker(0.5, ()->{robot.tureta_d();})
//                .lineToLinearHeading(new Pose2d(22, -13, Math.toRadians(0)))
//                .build();
//
//        TrajectorySequence intake2 = drive.trajectorySequenceBuilder(outake1.end())
//                .addTemporalMarker(0.1, ()->{robot.tureta_n();})
//                .addTemporalMarker(0.3, ()->{robot.con2();})
//                .lineToLinearHeading(new Pose2d(59, -8.5, Math.toRadians(0)))
//                .build();
//
//        TrajectorySequence outake2 = drive.trajectorySequenceBuilder(intake2.end())
//                .addTemporalMarker(0.01, ()->{robot.goMedium();})
//                .addTemporalMarker(0.5, ()->{robot.tureta_d();})
//                .lineToLinearHeading(new Pose2d(22, -13, Math.toRadians(0)))
//                .build();
//
//        TrajectorySequence intake3 = drive.trajectorySequenceBuilder(outake2.end())
//                .addTemporalMarker(0.01, ()->{robot.tureta_n();})
//                .addTemporalMarker(0.3, ()->{robot.con3();})
//                .lineToLinearHeading(new Pose2d(59, -10, Math.toRadians(0)))
//                .build();
//
//        TrajectorySequence outake3 = drive.trajectorySequenceBuilder(intake3.end())
//                .addTemporalMarker(0.01, ()->{robot.goMedium();})
//                .addTemporalMarker(0.5, ()->{robot.tureta_d();})
//                .lineToLinearHeading(new Pose2d(22, -14.5, Math.toRadians(0)))
//                .build();
//
//        TrajectorySequence intake4 = drive.trajectorySequenceBuilder(outake3.end())
//                .addTemporalMarker(0.1, ()->{robot.tureta_n();})
//                .addTemporalMarker(0.3, ()->{robot.con4();})
//                .lineToLinearHeading(new Pose2d(59.5, -10.5, Math.toRadians(0)))
//                .build();
//
//        TrajectorySequence outake4 = drive.trajectorySequenceBuilder(intake4.end())
//                .addTemporalMarker(0.01, ()->{robot.goMedium();})
//                .addTemporalMarker(0.5, ()->{robot.tureta_d();})
//                .lineToLinearHeading(new Pose2d(23, -15.5, Math.toRadians(0)))
//                .build();
//
//
//        TrajectorySequence park1 = drive.trajectorySequenceBuilder(outake4.end())
//                .addTemporalMarker(0.3, ()->{robot.tureta_n();})
//                .addTemporalMarker(0.8, ()->{robot.goDown();})
//                .lineToLinearHeading(new Pose2d(12, -15, Math.toRadians(0)))
//                .build();
//
//        TrajectorySequence park2 = drive.trajectorySequenceBuilder(outake4.end())
//                .addTemporalMarker(0.3, ()->{robot.tureta_n();})
//                .addTemporalMarker(0.8, ()->{robot.goDown();})
//                .lineToLinearHeading(new Pose2d(35, -15, Math.toRadians(0)))
//                .build();
//
//        TrajectorySequence park3 = drive.trajectorySequenceBuilder(outake4.end())
//                .addTemporalMarker(0.3, ()->{robot.tureta_n();})
//                .addTemporalMarker(0.8, ()->{robot.goDown();})
//                .lineToLinearHeading(new Pose2d(53, -15, Math.toRadians(0)))
//                .build();
//
//        robot.servo_inchis();
//        robot.tureta_n();
//        robot.hover_slider();
//
//
//        while (!isStarted() && !isStopRequested())
//        {
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if(currentDetections.size() != 0)
//            {
//                boolean tagFound = false;
//
//                for(AprilTagDetection tag : currentDetections)
//                {
//                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
//                    {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//
//                if(tagFound)
//                {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                }
//                else
//                {
//                    telemetry.addLine("Don't see tag of interest :(");
//
//                    if(tagOfInterest == null)
//                    {
//                        telemetry.addLine("(The tag has never been seen)");
//                    }
//                    else
//                    {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//            }
//            else
//            {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if(tagOfInterest == null)
//                {
//                    telemetry.addLine("(The tag has never been seen)");
//                }
//                else
//                {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//
//            }
//
//            telemetry.update();
//            sleep(20);
//        }
//
//        if(tagOfInterest != null)
//        {
//            telemetry.addLine("Tag snapshot:\n");
//            tagToTelemetry(tagOfInterest);
//            telemetry.update();
//        }
//        else
//        {
//            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//            telemetry.update();
//        }
//
//        //PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)
//
//
//        drive.followTrajectorySequence(preload);
//        sleep(200);
//        robot.servo_deschis();
//        sleep(300);
//        drive.followTrajectorySequence(preintake);
//        drive.followTrajectorySequence(intake1);
//        sleep(100);
//        robot.servo_inchis();
//        sleep(600);
//        drive.followTrajectorySequence(outake1);
//        sleep(300);
//        robot.servo_deschis();
//        sleep(700);
//        drive.followTrajectorySequence(intake2);
//        sleep(100);
//        robot.servo_inchis();
//        sleep(600);
//        drive.followTrajectorySequence(outake2);
//        sleep(300);
//        robot.servo_deschis();
//        sleep(700);
//        drive.followTrajectorySequence(intake3);
//        sleep(100);
//        robot.servo_inchis();
//        sleep(600);
//        drive.followTrajectorySequence(outake3);
//        sleep(300);
//        robot.servo_deschis();
//        sleep(400);
//        drive.followTrajectorySequence(intake4);
//        sleep(100);
//        robot.servo_inchis();
//        sleep(600);
//        drive.followTrajectorySequence(outake4);
//        sleep(300);
//        robot.servo_deschis();
//        sleep(700);
//
//
//
//        if(tagOfInterest == null){
//            drive.followTrajectorySequence(park2);
//        }else{
//            switch(tagOfInterest.id){
//                case 1:
//                    drive.followTrajectorySequence(park1);
//
//                    break;
//                case 2:
//                    drive.followTrajectorySequence(park2);
//                    break;
//                case 3:
//                    drive.followTrajectorySequence(park3);
//
//                    break;
//            }
//        }
//
//        sleep(200);
//
//    }
//
//    void tagToTelemetry(AprilTagDetection detection)
//    {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
////        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
////        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
////        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
////        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
////        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
////        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }
//}