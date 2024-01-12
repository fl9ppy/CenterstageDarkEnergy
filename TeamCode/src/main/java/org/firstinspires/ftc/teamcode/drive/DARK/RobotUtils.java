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
@Config
public class RobotUtils {

    public DcMotor slider1;
    public DcMotor slider2;
    public Servo lansator;
    public ServoImplEx brat1;
    public ServoImplEx brat2;
    public DcMotor intake;
    public DcMotor agatator;
    public Servo outake_front;
    public RevColorSensorV3 sensor_outake_back;
    public RevColorSensorV3 sensor_outake_front;
    public double lansator_lansare = 0.39;
    public double lansator_tragere = 0.7;
    public static double pos_servo_outake_inchis = 0.110;
    public static double pos_servo_outake_deschis = 0.20;
    public static double brat_sus = 0.37;
    public static double brat_jos = 0;
    public static int slidere_up = 0;
    public static int slidere_down = 0;

    //Trebuie aleasa cu robotu pornit, da mi-e prea lene sa il pornesc
    public  int safe_poz = 422;



     public RobotUtils(HardwareMap hardwareMap){
      slider1 = hardwareMap.get(DcMotor.class, "slider1");
      slider2 = hardwareMap.get(DcMotor.class,"slider2");
      outake_front = hardwareMap.get(Servo.class, "outake_front");
      lansator = hardwareMap.get(Servo.class, "lansator");
      brat1 = hardwareMap.get(ServoImplEx.class, "brat1");
      brat2 = hardwareMap.get(ServoImplEx.class, "brat2");
      intake = hardwareMap.get(DcMotor.class, "intake");
      agatator = hardwareMap.get(DcMotor.class,"agatator");
      sensor_outake_back = hardwareMap.get(RevColorSensorV3.class, "sensor_outake_back");
      sensor_outake_front = hardwareMap.get(RevColorSensorV3.class, "sensor_outake_front");

      slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      slider1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      slider2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      slider1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      slider2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      brat2.setDirection(Servo.Direction.REVERSE);

      brat1.setPwmRange(new PwmControl.PwmRange(505, 2495));
      brat2.setPwmRange(new PwmControl.PwmRange(505, 2495));
     }

     public void setSliderPositions(int position){
         slider1.setTargetPosition(position);
         slider2.setTargetPosition(-position);
     }

     public void goSliderToPosition(int position, double power) {

        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         // Ensure that power is positive.
        double absPower = Math.abs(power);

        // Get the current position of the slider.
        int currentPos = slider1.getCurrentPosition();

        // Set the target position of both slider motors.
        setSliderPositions(position);

        // Set the run mode of both slider motors to RUN_TO_POSITION.

        if (currentPos > position) {
            // If the current position is higher than the target position, move the sliders down.
            slider1.setPower(absPower);
            slider2.setPower(-absPower);
        }
        else if (currentPos < position) {
            // If the current position is lower than the target position, move the sliders up.
            slider1.setPower(-absPower);
            slider2.setPower(absPower);
        }
        // If the current position is already at the target position, the sliders do not need to move.
    }
    public void bratUp() {
            brat1.setPosition(brat_sus);
            brat2.setPosition(brat_sus);
    }
    public void bratDown(){
            brat1.setPosition(brat_jos);
            brat2.setPosition(brat_jos);
    }

    public boolean farEnough(){
         if(slider1.getCurrentPosition()>=safe_poz) return true;
         return false;
    }

    public boolean hasDetected(){
         if(sensor_outake_front.getDistance(DistanceUnit.MM)>=10 && sensor_outake_back.getDistance(DistanceUnit.MM)>=10)
             return true;
         return false;
    }

    public void ziuaimpingerii(){lansator.setPosition(lansator_lansare);}
    public void ziuatragerii(){lansator.setPosition(lansator_tragere);}

    public void intakerotire(){ intake.setPower(0.85);}
    public void intakeoprire(){ intake.setPower(0);}
    public void portita_inchisa(){ outake_front.setPosition(pos_servo_outake_deschis);}
    public void portita_deschis(){ outake_front.setPosition(pos_servo_outake_inchis);}

    public void slider_up(){
       slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         goSliderToPosition(-200, 0.7);
    }

    public void slider_down(){
        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         goSliderToPosition(10, 0.7);
    }
}
