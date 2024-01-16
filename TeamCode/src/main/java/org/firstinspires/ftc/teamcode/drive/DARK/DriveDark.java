package org.firstinspires.ftc.teamcode.drive.DARK;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="DriveDark",group = "LULU_SI_ARMON")
@Config
public class DriveDark extends LinearOpMode {
    private RobotUtils robot;
    private double loopTime;
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

            //Slidere
            if(gamepad2.left_trigger >= 0.3) {
                robot.slider1.setPower(0.75);
                robot.slider2.setPower(-0.75);
            } else if(gamepad2.right_trigger >= 0.3) {
                robot.slider1.setPower(-0.75);
                robot.slider2.setPower(0.75);
            } else {
                robot.slider1.setPower(0);
                robot.slider2.setPower(0);
            }

            //Intake
            if(gamepad2.right_bumper) robot.intake.setPower(0.75);
            else if(gamepad2.left_bumper) robot.intake.setPower(-0.75);
            else robot.intake.setPower(0);


            telemetry.addData("slider1: ", robot.slider1.getCurrentPosition());
            telemetry.addData("slider2: ", robot.slider2.getCurrentPosition());
            telemetry.addData("Mod sasiu: ", currentMode.toString());

            double loop = System.nanoTime();
            telemetry.addData("hz: ", 1000000000 / (loop - loopTime));
            loopTime = loop;

            telemetry.update();
            drive.update();
        }
    }
}




