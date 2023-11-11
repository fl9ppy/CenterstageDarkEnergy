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

@TeleOp(name="Drive",group = "teleop")
@Config
public class drive extends LinearOpMode {

    private RobotUtils robot;


    enum Modedrive {
        DRIVER_CONTROL,
        TURBO_CONTROL,
        PRECISION_CONTROL
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
                                    -gamepad1.left_stick_y/2.5,
                                    -gamepad1.left_stick_x/2.5,
                                    -gamepad1.right_stick_x/2.5
                            )
                    );

                    if (gamepad1.right_trigger>0.3) {

                        currentMode = Modedrive.TURBO_CONTROL;
                    }
                    if (gamepad1.left_trigger>0.3) {

                        currentMode = Modedrive.PRECISION_CONTROL;
                    }
                    break;
                case TURBO_CONTROL:

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y/1.5,
                                    -gamepad1.left_stick_x/1.5,
                                    -gamepad1.right_stick_x/1.5
                            )
                    );

                    if (gamepad1.right_trigger==0) {

                        currentMode = Modedrive.DRIVER_CONTROL;
                    }
                    break;
                case PRECISION_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y/4,
                                    -gamepad1.left_stick_x/4,
                                    -gamepad1.right_stick_x/4
                            )
                    );

                    if (gamepad1.left_trigger==0) {

                        currentMode = Modedrive.DRIVER_CONTROL;
                    }
                    break;

            }
            drive.update();

            switch (sliderMode){
                case DOWN:
                    robot.goLow();

                    if(gamepad1.dpad_left) sliderMode = sliderMode.IDLE;
                    if(gamepad1.dpad_right) sliderMode = sliderMode.MANUAL;
                    break;

                case IDLE:
                    robot.slider1.setPower(0);
                    robot.slider2.setPower(0);

                    if(gamepad1.dpad_down) sliderMode = sliderMode.DOWN;
                    if(gamepad1.dpad_right) sliderMode = sliderMode.MANUAL;
                    break;

// TODO: case manual si faza cu encoderele;
                // TODO: cand se ridica bratul se deschide gheara automat;

//                case MANUAL:
//                    robot.slider1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.slider2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    if(gamepad1.left_bumper)
//                    {
//                        robot.slider.setPower(-0.75);
//                        robot.slider2.setPower(-0.75);
//                    }
//                    else if(gamepad1.right_bumper)
//                    {
//                        robot.slider.setPower(0.75);
//                        robot.slider2.setPower(0.75);
//                    }
//                    else
//                    {
//                        robot.slider.setPower(0);
//                        robot.slider2.setPower(0);
//                    }
//                    if(gamepad1.triangle)
//                    {
//                        robot.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        robot.slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        robot.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        robot.slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        currentModeSlider = ModeSlider.IDLE;
//                    }
            }

       if(gamepad2.square){
           robot.gheara.setPosition(robot.gheara_deschisa);}

       if(gamepad2.x){
           robot.gheara.setPosition(robot.gheara_inchisa);}
       }

       if(gamepad1.triangle) {
           robot.lansator.setPosition(robot.lansator_lansare);
       }

       if(gamepad1.circle){
           robot.lansator.setPosition(robot.lansator_tragere);
       }

       if(gamepad1.x){
           robot.brat1.setPosition(robot.brat_sus);
           robot.brat2.setPosition(robot.brat_sus);
       }

       if(gamepad1.square){
           robot.brat1.setPosition(robot.brat_jos);
           robot.brat2.setPosition(robot.brat_jos);
       }

       if(gamepad2.left_trigger){
           robot.intake.setPower(robot.intake_power);
       }

       if(gamepad2.right_trigger){
           robot.agatator.setPower(robot.putere_agatator);
       }

       telemetry.addData("brat2", robot.brat2.getPosition());
         telemetry.addData("brat1", robot.brat1.getPosition());
            telemetry.addData("slider1 ", robot.slider1.getCurrentPosition());
        telemetry.addData("slider2", robot.slider2.getCurrentPosition());
        telemetry.addData("gheara", robot.gheara.getPosition());


        telemetry.update();
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




