//package org.firstinspires.ftc.teamcode.drive.DARK;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//
//@TeleOp(name = "CustomTeleOp", group = "teleop")
//@Config
//public class CustomTeleOp extends LinearOpMode {
//
//    private enum DriveMode {
//        DRIVER_CONTROL,
//        TURBO,
//        PRECISION
//    }
//
//    private enum SliderMode {
//        DOWN,
//        IDLE,
//        MANUAL
//    }
//
//    private DriveMode currentMode = DriveMode.DRIVER_CONTROL;
//    private SliderMode sliderMode = SliderMode.DOWN;
//
//    private SampleMecanumDrive drive;
//    private RobotUtils robot;
//    private ElapsedTime rotireTimer = new ElapsedTime();
//    private ElapsedTime armTimer = new ElapsedTime();
//    private ElapsedTime ghearaTimer = new ElapsedTime();
//    private ElapsedTime dropTimer = new ElapsedTime();
//    private ElapsedTime initialRiseTimer = new ElapsedTime();
//
//    private boolean movedSlider = false;
//    private boolean ghearaDeschisa = false;
//    private boolean robotGoDown = false;
//    private boolean robotArmDown = false;
//    private boolean ready = false;
//    private boolean hasDetectedObj = false;
//    private boolean armHovered = false;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot = new RobotUtils(hardwareMap);
//        drive = new SampleMecanumDrive(hardwareMap);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive() && !isStopRequested()) {
//            switch (currentMode) {
//                case DRIVER_CONTROL:
//                    drive.setWeightedDrivePower(
//                            new Pose2d(
//                                    -gamepad1.left_stick_y / 1.5,
//                                    -gamepad1.left_stick_x / 1.5,
//                                    -gamepad1.right_stick_x / 1.5
//                            )
//                    );
//
//                    if (gamepad1.right_trigger > 0.3) {
//                        currentMode = DriveMode.TURBO;
//                    }
//                    if (gamepad1.left_trigger > 0.3) {
//                        currentMode = DriveMode.PRECISION;
//                    }
//                    break;
//                case TURBO:
//                    drive.setWeightedDrivePower(
//                            new Pose2d(
//                                    -gamepad1.left_stick_y,
//                                    -gamepad1.left_stick_x,
//                                    -gamepad1.right_stick_x
//                            )
//                    );
//
//                    if (gamepad1.right_trigger == 0) {
//                        currentMode = DriveMode.DRIVER_CONTROL;
//                    }
//                    break;
//                case PRECISION:
//                    drive.setWeightedDrivePower(
//                            new Pose2d(
//                                    -gamepad1.left_stick_y / 3,
//                                    -gamepad1.left_stick_x / 3,
//                                    -gamepad1.right_stick_x / 3
//                            )
//                    );
//
//                    if (gamepad1.left_trigger == 0) {
//                        currentMode = DriveMode.DRIVER_CONTROL;
//                    }
//            }
//
//            switch (sliderMode) {
//                case DOWN:
//                    robot.goLow();
//
//                    if (gamepad1.dpad_left) sliderMode = SliderMode.IDLE;
//                    if (gamepad1.dpad_right) sliderMode = SliderMode.MANUAL;
//                    break;
//                case IDLE:
//                    robot.slider1.setPower(0);
//                    robot.slider2.setPower(0);
//
//                    if (gamepad1.dpad_down) sliderMode = SliderMode.DOWN;
//                    if (gamepad1.dpad_right) sliderMode = SliderMode.MANUAL;
//                    break;
//                case MANUAL:
//                    robot.slider1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.slider2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//                    if (gamepad1.left_bumper) {
//                        robot.slider1.setPower(-0.75);
//                        robot.slider2.setPower(-0.75);
//                    } else if (gamepad1.right_bumper) {
//                        robot.slider1.setPower(0.75);
//                        robot.slider2.setPower(0.75);
//                    } else {
//                        robot.slider1.setPower(0);
//                        robot.slider2.setPower(0);
//                    }
//
//                    if (gamepad1.triangle) {
//                        robot.slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        robot.slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        robot.slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        robot.slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        sliderMode = SliderMode.IDLE;
//                    }
//            }
//
//            if (gamepad2.square) {
//                robot.gheara_open();
//            }
//
//            if (gamepad2.x) {
//                robot.gheara_closed();
//            }
//
//            int press_count = 1;
//
//            if (gamepad1.right_bumper) press_count += 1;
//
//            if (press_count % 2 == 0) robot.intake.setPower(0.5);
//            else robot.intake.setPower(0);
//
//            int press_count2 = 1;
//
//            if (gamepad1.left_bumper) press_count2 += 1;
//
//            if (press_count2 % 2 == 0) robot.intake.setPower(-0.5);
//            else robot.intake.setPower(0);
//
//            if (gamepad1.x) {
//                robot.brat1.setPosition(robot.brat_sus);
//                robot.brat2.setPosition(robot.brat_sus);
//            }
//
//            if (gamepad1.square) {
//                robot.brat1.setPosition(robot.brat_jos);
//                robot.brat2.setPosition(robot.brat_jos);
//            }
//
//            handleAutoMode();
//
//            telemetry.addData("brat2", robot.brat2.getPosition());
//            telemetry.addData("brat1", robot.brat1.getPosition());
//            telemetry.addData("slider1 ", robot.slider1.getCurrentPosition());
//            telemetry.addData("slider2", robot.slider2.getCurrentPosition());
//            telemetry.addData("slider2", robot.gheara.getPosition());
//            telemetry.update();
//        }
//    }
//
//    private void handleAutoMode() {
//        if (currentMode != DriveMode.MANUAL) {
//            if (gamepad2.right_bumper) currentMode = DriveMode.MANUAL;
//
//            if (gamepad1.left_bumper) ready = true;
//
//            boolean detectOk = rotireTimer.time() > 700;
//
//            if ((robot.hasDetectedObject() && detectOk && !hasDetectedObj) || gamepad1.right_stick_button) {
//                robot.inchideGheara();
//                hasDetectedObj = true;
//                ghearaTimer.reset();
//                initialRiseTimer.reset();
//            }
//
//            if (hasDetectedObj && initialRiseTimer.time() < 200 && initialRiseTimer.time() > 150 && !armHovered) {
//                robot.armHover();
//                armHovered = true;
//            }
//
//            boolean ok = (ghearaTimer.time() > 500);
//
//            if (ok && ready) {
//                movedSlider = true;
//                switch (robot.getPoleType()) {
//                    case HIGH:
//                        robot.goUpSlider();
//                        robot.goUp();
//                        break;
//                    case MEDIUM:
//                        robot.goMediumSlider();
//                        robot.goUp();
//                        break;
//                    case LOW:
//                        robot.goLowSlider();
//                        robot.goUp();
//                        break;
//                    case GROUND:
//                        robot.armHover();
//                        break;
//                }
//            }
//
//            if (gamepad1.square) {
//                robot.deschideGheara();
//                if (movedSlider) {
//                    ghearaDeschisa = true;
//                    rotireTimer.reset();
//                    ghearaTimer.reset();
//                }
//            }
//
//            if (ghearaDeschisa && ghearaTimer.time() > 300) {
//                ghearaDeschisa = false;
//                movedSlider = false;
//                dropTimer.reset();
//                armTimer.reset();
//                robotArmDown = true;
//                robotGoDown = true;
//                hasDetectedObj = false;
//                armHovered = false;
//            }
//
//            if (dropTimer.time() > 300 && robotGoDown) {
//                robotGoDown = false;
//                robot.goDownSlider();
//                dropTimer.reset();
//                ready = false;
//                robot.goDown();
//            }
//
//            boolean shouldChange = movedSlider;
//
//            if (gamepad1.dpad_up)
//                robot.setPoleType(RobotUtils.PoleType.HIGH);
//            else if (gamepad1.dpad_down) {
//                robot.setPoleType(RobotUtils.PoleType.MEDIUM);
//            } else if (gamepad1.dpad_left)
//                robot.setPoleType(RobotUtils.PoleType.LOW);
//            else if (gamepad1.dpad_right)
//                robot.setPoleType(RobotUtils.PoleType.GROUND);
//            else {
//                shouldChange = false;
//            }
//
//            if (shouldChange && ready) {
//                switch (robot.getPoleType()) {
//                    case HIGH:
//                        robot.goUpSlider();
//                        robot.goUp();
//                        break;
//                    case MEDIUM:
//                        robot.goMediumSlider();
//                        robot.goUp();
//                        break;
//                    case LOW:
//                        robot.goLowSlider();
//                        robot.goUp();
//                        break;
//                    case GROUND:
//                        robot.armHover();
//                        armTimer.reset();
//                        robotArmDown = true;
//                }
//            }
//
//            telemetry.addData("ready:", ready);
//            telemetry.addData("poletype:", robot.getPoleType().toString());
//            telemetry.addData("hasdetectedobj:", hasDetectedObj);
//            telemetry.addData("detectok: ", detectOk);
//            telemetry.addData("sensor: ", robot.hasDetectedObject());
//        } else {
//            if (currentMode == DriveMode.MANUAL) {
//                robot.sliderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.sliderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//                robot.sliderRight.setPower(-gamepad2.left_stick_y / 2);
//                robot.sliderLeft.setPower(gamepad2.left_stick_y / 2);
//
//                if (gamepad2.triangle) {
//                    robot.sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    robot.sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                    robot.sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                    currentMode = DriveMode.AUTO;
//                }
//            }
//        }
//    }
//}
