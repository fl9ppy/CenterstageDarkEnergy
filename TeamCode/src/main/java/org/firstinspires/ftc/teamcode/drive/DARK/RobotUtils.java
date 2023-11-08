package org.firstinspires.ftc.teamcode.drive.DARK;

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
    public Servo lansator;
    public Servo brat1;
    public Servo brat2;
    public Servo gheara;
    public DcMotor intake;

     public RobotUtils(HardwareMap hardwareMap){
      slider1 = hardwareMap.get(DcMotor.class, "slider1");
      slider2 = hardwareMap.get(DcMotor.class,"slider2");
      lansator = hardwareMap.get(Servo.class, "lansator");
      brat1 = hardwareMap.get(Servo.class, "brat1");
      brat2 = hardwareMap.get(Servo.class, "brat2");
      gheara = hardwareMap.get(Servo.class, "gheara");
      intake = hardwareMap.get(DcMotor.class, "intake");

      slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

     }



}
