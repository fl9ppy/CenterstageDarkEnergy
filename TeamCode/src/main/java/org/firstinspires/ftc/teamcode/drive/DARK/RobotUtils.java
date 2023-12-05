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
    public DcMotor agatator;
    public Servo carlig;
    //public RevColorSensorV3 sensor_auto;
    public static int gheara_deschisa = 1;
    public static int gheara_inchisa = 0;
    public static int slider_low =2413;
    public static int slider_down = 2413; //deloc ridicat
    public static int lansator_tragere = 0;
    public static int lansator_lansare = 0;
    public static int brat_sus = 0;
    public static int brat_jos = 0;
    public static double intake_power = 1;
    public static int putere_agatator = 0;
    public static int pozitie_carlig = 0;

    //Trebuie aleasa cu robotu pornit, da mi-e prea lene sa il pornesc
    public static int safe_poz = 0;



     public RobotUtils(HardwareMap hardwareMap){
      slider1 = hardwareMap.get(DcMotor.class, "slider1");
      slider2 = hardwareMap.get(DcMotor.class,"slider2");
      //lansator = hardwareMap.get(Servo.class, "lansator");
//      brat1 = hardwareMap.get(ServoImplEx.class, "brat1");
//      brat2 = hardwareMap.get(ServoImplEx.class, "brat2");
//      gheara = hardwareMap.get(Servo.class, "gheara");
      intake = hardwareMap.get(DcMotor.class, "intake");
//      agatator = hardwareMap.get(DcMotor.class,"agatator");
//      carlig = hardwareMap.get(Servo.class, "carlig");
      //sensor_auto= hardwareMap.get(RevColorSensorV3.class, "sensor_auto");

      slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      slider1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     slider2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//      brat2.setDirection(Servo.Direction.REVERSE);

//      brat1.setPwmRange(new PwmControl.PwmRange(505, 2495));
//      brat2.setPwmRange(new PwmControl.PwmRange(505, 2495));
     }

     public void setSliderPositions(int position){
         slider1.setTargetPosition(position);
         slider2.setTargetPosition(-position);
     }

     public void goSliderToPosition(int position, double power) {
        // Ensure that power is positive.
        double absPower = Math.abs(power);

        // Get the current position of the slider.
        int currentPos = slider1.getCurrentPosition();

        // Set the target position of both slider motors.
        setSliderPositions(position);

        // Set the run mode of both slider motors to RUN_TO_POSITION.
        slider1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (currentPos > position) {
            // If the current position is higher than the target position, move the sliders down.
            slider1.setPower(-absPower);
            slider2.setPower(absPower);
        }
        else if (currentPos < position) {
            // If the current position is lower than the target position, move the sliders up.
            slider1.setPower(absPower);
            slider2.setPower(-absPower);
        }
        // If the current position is already at the target position, the sliders do not need to move.
    }
    public void goLow(){
        goSliderToPosition(slider_low, 0.6);
    }
    //1500
    //1600

    public void goDown(){
        goSliderToPosition(slider_down, 0.6);
    }


    //Poate mai incepi sa adaugi ceva comentarii ca sa inteleg si eu ce ii pe aici??? xDDD
//    public void fullintake(){
//         intake.setPower(intake_power);
//         brat1.setPosition(brat_jos);
//         brat2.setPosition(brat_jos);
    //     gheara.setPosition(gheara_deschisa);
//    }

    ///Aceste functii sunt beta si doar asa ca sa testam ideea cu flipul in functie de pozitie
//    public void bratUp(){
//        brat1.setPosition(brat_sus);
//        brat2.setPosition(brat_sus);
//    }
//
//    public void bratDown(){
//        brat1.setPosition(brat_jos);
//        brat2.setPosition(brat_jos);
//    }

    boolean farEnough(){
         if(slider1.getCurrentPosition()>=safe_poz)
             return true;
         return false;
    }
}
