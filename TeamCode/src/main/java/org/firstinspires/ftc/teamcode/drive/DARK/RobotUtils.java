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
    public ServoImplEx brat1;
    public ServoImplEx brat2;
    public Servo gheara;
    public DcMotor intake;
    public DcMotor agatator;
    public Servo carlig;
    public static double gheara_deschisa = 0.0;
    public static double gheara_inchisa =0.0;
    public static int slider_low =0;
    public static int slider_down = 0; //deloc ridicat
    public static double power_slider1_down = 1;
    public static double power_slider2_down = -1;
    public static int lansator_tragere = 0;
    public static int lansator_lansare = 0;
    public static int brat_sus = 0;
    public static int brat_jos = 0;
    public static double intake_power = 0;
    public static int putere_agatator = 0;
    public static int pozitie_carlig = 0;



     public RobotUtils(HardwareMap hardwareMap){
      slider1 = hardwareMap.get(DcMotor.class, "slider1");
      slider2 = hardwareMap.get(DcMotor.class,"slider2");
      lansator = hardwareMap.get(Servo.class, "lansator");
      brat1 = hardwareMap.get(ServoImplEx.class, "brat1");
      brat2 = hardwareMap.get(ServoImplEx.class, "brat2");
      gheara = hardwareMap.get(Servo.class, "gheara");
      intake = hardwareMap.get(DcMotor.class, "intake");
      agatator = hardwareMap.get(DcMotor.class,"agatator");
      carlig = hardwareMap.get(Servo.class, "carlig");

      slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

     }


    public void goLow(){
        slider1.setTargetPosition(slider_low);
        slider2.setTargetPosition(slider_low);
        slider1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider1.setPower(power_slider1_down);
        slider2.setPower(power_slider2_down);
    }

    public void fullintake(){
         intake.setPower(intake_power);
        brat1.setPosition(brat_jos);
        brat2.setPosition(brat_jos);
        gheara.setPosition(gheara_deschisa);
    }

    public void Gheara_deschisa(){
      gheara.setPosition(gheara_deschisa);
    }
    public void Gheara_inchisa(){
         gheara.setPosition(gheara_inchisa);
    }

}
