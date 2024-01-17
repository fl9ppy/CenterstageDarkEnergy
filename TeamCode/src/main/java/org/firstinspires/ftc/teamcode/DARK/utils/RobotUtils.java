package org.firstinspires.ftc.teamcode.DARK.utils;

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

public class RobotUtils {
    public DcMotor slider1;
    public DcMotor slider2;

    public Servo plane;

    public ServoImplEx axon1;
    public ServoImplEx axon2;

    public Servo outake;
    public DcMotor intake;

    public RevColorSensorV3 sensor1;
    public RevColorSensorV3 sensor2;

    public static double outake_open = 0.6;
    public static double outake_close = 0.2;

    public static double plane_launch_pos = 0;
    public static double plane_armed_pos = 0;

    public static double axon_up_pos = 0.33;
    public static double axon_down_pos = 0;

    public static int slider_up = 0;
    public static int slider_down = 0;

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

      slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      axon1.setDirection(Servo.Direction.REVERSE);

      axon1.setPwmRange(new PwmControl.PwmRange(505, 2495));
      axon2.setPwmRange(new PwmControl.PwmRange(505, 2495));
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
    public void axonUp(){
        axon1.setPosition(axon_up_pos);
        axon2.setPosition(axon_up_pos);
    }
    public void axonDown() {
        axon1.setPosition(axon_down_pos);
        axon2.setPosition(axon_down_pos);
    }
    public void planeLaunch() {plane.setPosition(plane_launch_pos);}
    public void planeArmed() {plane.setPosition(plane_armed_pos);}
    public void outake_open() {outake.setPosition(outake_open);}
    public void outake_close() {outake.setPosition(outake_close);}
    public void slider_up() {goSliderToPosition(slider_up, 0.7);}
    public void slider_down() {goSliderToPosition(slider_down, 0.7);}
}
