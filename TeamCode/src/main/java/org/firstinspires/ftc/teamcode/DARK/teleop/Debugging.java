package org.firstinspires.ftc.teamcode.DARK.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DARK.utils.RobotUtils;
import org.firstinspires.ftc.teamcode.DARK.utils.SampleMecanumDrive;

@TeleOp(name="Debugging", group = "Debugging")
@Config

public class Debugging extends LinearOpMode {
    private RobotUtils robot;
    ElapsedTime timer = new ElapsedTime();
    private static final double DRIVE_SCALE = 1.7, TURBO_SCALE = 1, PRECISION_SCALE = 4;
    boolean buttonWasPressed = false;
    int cnt = 0;
    public static double loopTime = 0;
    public boolean holdSliders = false;

    enum Modedrive {
        DRIVER_CONTROL,
        TURBO,
        PRECISION
    }

    Modedrive currentMode = Modedrive.DRIVER_CONTROL;

    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robot = new RobotUtils(hardwareMap);

        robot.slider1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.slider2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.extension_up();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            /*-------------------------P1-------------------------*/

            //Chassis
            if (gamepad1.right_trigger != 0) currentMode = Modedrive.TURBO;
            else if (gamepad1.left_trigger != 0) currentMode = Modedrive.PRECISION;
            else currentMode = Modedrive.DRIVER_CONTROL;

            double driveScale = (currentMode == Modedrive.TURBO) ? TURBO_SCALE : (currentMode == Modedrive.PRECISION) ? PRECISION_SCALE : DRIVE_SCALE;
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y / driveScale,
                            -gamepad1.left_stick_x / driveScale,
                            -gamepad1.right_stick_x / driveScale
                    )
            );

            //drive.update();

            //outake
            if (gamepad1.right_bumper) robot.outake_open();
            if (gamepad1.right_stick_button) robot.outake_drop();
            if (gamepad1.left_bumper) robot.outake_close();

            /*-------------------------P2-------------------------*/


            //Sliders
            if (gamepad2.right_trigger >= 0.3) {
                robot.slider1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slider2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                robot.slider1.setPower(1);
                robot.slider2.setPower(-1);

                holdSliders = false;

            } else if (gamepad2.left_trigger >= 0.3) {
                robot.slider1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slider2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                robot.slider1.setPower(-1);
                robot.slider2.setPower(1);

                holdSliders = false;

            } else {
                if(!holdSliders) {
                    int currentPos2 = robot.slider2.getCurrentPosition();

                    robot.slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    robot.goSliderToPosition(currentPos2, 0.5);
                }
                holdSliders = true;
            }

            //Intake
            if (gamepad2.left_bumper) robot.intake_power(); // inghite
            else if (gamepad2.right_bumper) robot.inverse_intake_power(); // scuipa
            else robot.intake.setPower(0);

            //Arm
            if (gamepad2.square) robot.axonUp();
            if (gamepad2.circle) robot.axonDown();

            //Plane
            if (gamepad2.triangle) robot.planeLaunch();
            if (gamepad2.cross) robot.planeArmed();

            //Intake extension
            if (gamepad2.dpad_up) {
                robot.extension_up();
                cnt=0;
            }
            if (gamepad2.dpad_down){
                robot.extension_down();
                cnt = 0;
            }

            //Stack controls

            if (gamepad2.dpad_right)
                if (!buttonWasPressed) {
                    if(timer.milliseconds() > 100) {
                        buttonWasPressed = true;
                        timer.reset();
                    }
                }
            else buttonWasPressed = false;

            if (buttonWasPressed) {
                cnt++;
                buttonWasPressed = false;
            }

            if (cnt == 1) robot.intake_extension.setPosition(0.15);
            if (cnt == 2) robot.intake_extension.setPosition(0.1);
            if (cnt == 3) robot.intake_extension.setPosition(0.05);
            if (cnt == 4) robot.intake_extension.setPosition(0.01);
            if (cnt == 5) {
                robot.intake_extension.setPosition(0);
                cnt = 0;
            }

            /*-------------------------BOTH-------------------------*/

            //Pixel detection

            boolean ok = true;
            if(robot.hasDetected()){
                gamepad1.rumble(500);
                gamepad2.rumble(500);
                if(robot.slider2.getCurrentPosition() >= 100 && robot.slider2.getCurrentPosition() <= 150) robot.outake_close();
            }

            /*-------------------------TELEMETRY-------------------------*/
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.addData("has detected both pixel: ", robot.hasDetected());
            telemetry.addData("pressed: ", cnt);
            telemetry.addData("slider1: ", robot.slider1.getCurrentPosition());
            telemetry.addData("slider2: ", robot.slider2.getCurrentPosition());
            telemetry.addData("intake_extension: ", robot.intake_extension.getPosition());
            telemetry.addData("axon1: ", robot.axon1.getPosition());
            telemetry.addData("axon2: ", robot.axon2.getPosition());
            telemetry.addData("cuva: ", robot.outake.getPosition());
            telemetry.addData("avion: ", robot.plane.getPosition());
            telemetry.update();
        }
    }
}




