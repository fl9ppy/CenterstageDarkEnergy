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


    enum Mode {
        TURBO,
        PRECISION,
        DRIVER_CONTROL
    }

    enum Mode2 {
        DOWN,
        HIGH,
        MID,
        LOW
    }

    Mode2 sliderMode = Mode2.DOWN;
    Mode currentMode = Mode.DRIVER_CONTROL;

    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robot = new RobotUtils(hardwareMap);

        //TODO: robot.flip_start_pos(0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            switch (currentMode) {

                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y/1.7,
                                    -gamepad1.left_stick_x/1.7,
                                    -gamepad1.right_stick_x/1.7
                            )
                    );

                    if (gamepad1.right_trigger!=0) currentMode = Mode.TURBO;
                    if (gamepad1.left_trigger!=0) currentMode = Mode.PRECISION;

                    break;

                case TURBO:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    if (gamepad1.right_trigger==0) currentMode = Mode.DRIVER_CONTROL;

                    break;

                case PRECISION:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y/4,
                                    -gamepad1.left_stick_x/4,
                                    -gamepad1.right_stick_x/4
                            )
                    );

                    if(gamepad1.left_trigger==0) currentMode = Mode.DRIVER_CONTROL;

                    break;
            }
            drive.update();

        }
        telemetry.addData("slider1 ", robot.slider1.getCurrentPosition());
        telemetry.addData("slider2", robot.slider2.getCurrentPosition());
        telemetry.addData("gheara", robot.gheara.getPosition());


        telemetry.update();
    }
}