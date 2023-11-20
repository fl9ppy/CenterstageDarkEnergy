package org.firstinspires.ftc.teamcode.drive.DARK;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="DriveDark",group = "teleop")
@Config
public class DriveDark extends LinearOpMode {

    private RobotUtils robot;


    enum Modedrive {
        DRIVER_CONTROL,
        TURBO,
        PRECISION
    }

    enum Mode2Slider {
        DOWN,
        IDLE,
        MANUAL
    }

    Mode2Slider sliderMode = Mode2Slider.DOWN;
    Modedrive currentMode = Modedrive.DRIVER_CONTROL;

    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        robot = new RobotUtils(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            switch (currentMode) {

                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y / 1.7,
                                    -gamepad1.left_stick_x / 1.7,
                                    -gamepad1.right_stick_x / 1.7
                            )
                    );

                    if (gamepad1.right_trigger != 0) currentMode = currentMode.TURBO;
                    if (gamepad1.left_trigger != 0) currentMode = currentMode.PRECISION;

                    break;

                case TURBO:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    if (gamepad1.right_trigger == 0) currentMode = currentMode.DRIVER_CONTROL;

                    break;

                case PRECISION:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y / 4,
                                    -gamepad1.left_stick_x / 4,
                                    -gamepad1.right_stick_x / 4
                            )
                    );

                    if (gamepad1.left_trigger == 0) currentMode = currentMode.DRIVER_CONTROL;

                    break;
            }


            switch (sliderMode) {
                case DOWN:
                    robot.goLow();

                    if (gamepad1.dpad_left) sliderMode = sliderMode.IDLE;
                    if (gamepad1.dpad_right) sliderMode = sliderMode.MANUAL;
                    break;

                case IDLE:
                    robot.slider1.setPower(0);
                    robot.slider2.setPower(0);

                    if (gamepad1.dpad_down) sliderMode = sliderMode.DOWN;
                    if (gamepad1.dpad_left) sliderMode = sliderMode.MANUAL;
                    break;

                case MANUAL:
                    robot.slider1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.slider2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    if (gamepad1.left_bumper) {
                        robot.slider1.setPower(0.75);
                        robot.slider2.setPower(-0.75);
                    } else if (gamepad1.right_bumper) {
                        robot.slider1.setPower(-0.75);
                        robot.slider2.setPower(0.75);
                    } else {
                        robot.slider1.setPower(0);
                        robot.slider2.setPower(0);

                        if (gamepad1.dpad_right) {
                            robot.goLow();
                        } else if (gamepad1.dpad_down) {
                            robot.goDown();
                        }
                    }

                    if (gamepad1.triangle) {
                        robot.slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        sliderMode = Mode2Slider.IDLE;
                    }
                    break;
            }

            if (gamepad2.square) {robot.gheara_open();}
            if (gamepad2.x) {robot.gheara_closed();}


//       if(gamepad1.triangle) {
//           robot.lansator.setPosition(robot.lansator_lansare);
//       }
//
//       if(gamepad1.circle){
//           robot.lansator.setPosition(robot.lansator_tragere);
//       }
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

            if (gamepad1.x) {
                robot.brat1.setPosition(robot.brat_sus);
                robot.brat2.setPosition(robot.brat_sus);
            }

            if (gamepad1.square) {
                robot.brat1.setPosition(robot.brat_jos);
                robot.brat2.setPosition(robot.brat_jos);
            }

            telemetry.addData("brat2", robot.brat2.getPosition());
            telemetry.addData("brat1", robot.brat1.getPosition());
        telemetry.addData("mode", sliderMode.toString());
        telemetry.addData("slider2", robot.gheara.getPosition());
        telemetry.addData("slider1",robot.slider1.getCurrentPosition());
        telemetry.addData("slider2",robot.slider2.getCurrentPosition());


        telemetry.update();
        drive.update();}
    }
}









//**// Controale Armon
//pozitile slidere : high pe sageata sus;
//       mijloc left;
//       down high;
//       poz initiala cu cleste inapoim sageata down;
//       lansare avion y(galben);
//       cand ridici slider bratl de intoarce automat; a(verde)
//        motor cab se ridica sus; x merge in jos;
//
//  Controale Lulu
//          rotative rb
//            gheara inchidere x; a dai drumul
//           ridicare dpad up dpad down jos;
//
//




