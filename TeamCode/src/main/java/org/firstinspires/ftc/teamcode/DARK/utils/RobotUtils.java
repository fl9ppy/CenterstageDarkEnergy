package org.firstinspires.ftc.teamcode.DARK.utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Config

public class RobotUtils {
    public DcMotor slider1, slider2, intake;
    public Servo plane, outake;
    public ServoImplEx axon1, axon2, intake_extension;
    public RevColorSensorV3 sensor1, sensor2;
    public static double extension_up = 0.4, extension_down = 0, extension_stack = 0;
    public static double outake_close = 0.5, outake_open = 0.2, diff=0;
    public static double plane_launch_pos = 0.7, plane_armed_pos = 0;
    public static double axon_up_pos = 0.38, axon_down_pos = 0;
    public static int slider_up1 = -725, slider_up2= 725, slider_down1 = 14, slider_down2 = -14;

     public RobotUtils(HardwareMap hardwareMap){
      slider1 = hardwareMap.get(DcMotor.class, "slider1");
      slider2 = hardwareMap.get(DcMotor.class,"slider2");

      plane = hardwareMap.get(Servo.class, "plane");

      axon1 = hardwareMap.get(ServoImplEx.class, "axon1");
      axon2 = hardwareMap.get(ServoImplEx.class, "axon2");

      outake = hardwareMap.get(Servo.class, "outake");
      intake = hardwareMap.get(DcMotor.class, "intake");

      sensor1 = hardwareMap.get(RevColorSensorV3.class, "sensor1");
      sensor2 = hardwareMap.get(RevColorSensorV3.class, "sensor2");

      intake_extension = hardwareMap.get(ServoImplEx.class, "intake_extension");

      slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      axon1.setDirection(Servo.Direction.REVERSE);

      axon1.setPwmRange(new PwmControl.PwmRange(505, 2495));
      axon2.setPwmRange(new PwmControl.PwmRange(505, 2495));
      intake_extension.setPwmRange(new PwmControl.PwmRange(505, 2495));
     }

     public void setSliderPositions(int position){
         slider1.setTargetPosition(position);
         slider2.setTargetPosition(-position);
     }

     public void goSliderToPosition(int position, double power) {
        double absPower = Math.abs(power);

        int currentPos = slider1.getCurrentPosition();

        setSliderPositions(position);

        slider1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (currentPos > position) {
            slider1.setPower(-absPower);
            slider2.setPower(absPower);
        }
        else if (currentPos < position) {
            slider1.setPower(absPower);
            slider2.setPower(-absPower);
        }
    }

    public void slider_up(){
        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider1.setTargetPosition(slider_up1);
        slider2.setTargetPosition(slider_up2);
        slider1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider1.setPower(-0.75);
        slider2.setPower(0.75);
    }

    public void slider_down(){
        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider1.setTargetPosition(slider_down1);
        slider2.setTargetPosition(slider_down2);
        slider1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider1.setPower(0.75);
        slider2.setPower(-0.75);
    }
    public void axonUp(){
        axon1.setPosition(axon_up_pos);
        axon2.setPosition(axon_up_pos);
    }
    public void axonDown() {
        axon1.setPosition(axon_down_pos);
        axon2.setPosition(axon_down_pos);
    }

    public boolean hasDetected(){
         if(sensor1.getDistance(DistanceUnit.MM) <= 10 || sensor2.getDistance(DistanceUnit.MM) <= 10) return true;
         return false;
    }

    public void planeLaunch() {plane.setPosition(plane_launch_pos);}

    public void planeArmed() {plane.setPosition(plane_armed_pos);}

    public void outake_open() {outake.setPosition(outake_open);}

    public void outake_close() {outake.setPosition(outake_close);}

    public void extension_up() {intake_extension.setPosition(extension_up);}

    public void extension_down() {intake_extension.setPosition(extension_down);}

    public void intake_power() {intake.setPower(0.85);}

    public void inverse_intake_power() {intake.setPower(-0.75);}
}
