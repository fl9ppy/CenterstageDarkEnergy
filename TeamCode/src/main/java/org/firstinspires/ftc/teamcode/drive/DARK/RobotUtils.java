/***
 ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⣷⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⣿⣿⣿⣿⡷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 ⠀⠀⠀⠀⠀⠀⠀⠁⠀⢀⣴⣿⣿⣿⣿⡿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 ⠀⠀⠀⠀⠀⠀⠀⢀⣴⣿⣿⣿⣿⡿⠋⠀⠀⠀⠀⢀⣴⣷⣄⠀⠀⠀⠀⠀⠀⠀
 ⠀⠀⠀⠀⠀⠀⢴⣿⣿⣿⣿⣿⣏⠀⠀⠀⠀⢀⣴⣿⣿⣿⣿⣷⣄⠀⠀⠀⠀⠀
 ⠀⠀⠀⠀⠀⠀⠀⠈⠿⣿⣿⣿⣿⣷⣄⢀⣴⣿⣿⣿⣿⣿⣿⣿⣿⣷⣄⠀⠀⠀
 ⠀⢀⣴⣧⣄⠀⠀⠀⠀⠙⢻⣿⣿⣿⣿⣿⣿⣿⣿⡿⠋⠉⠿⣿⣿⣿⣿⣷⣄⠀
 ⢴⣿⣿⣿⣿⣶⣀⠀⠀⠀⠀⢘⣿⣿⣿⣿⣿⣿⣏⠀⠁⠀⠀⠉⠿⣿⣿⣿⣿⡷
 ⠀⠙⠿⣿⣿⣿⣿⣶⣄⢀⣴⣿⣿⣿⣿⣿⣿⣿⣿⣷⣄⠀⠀⠀⠀⠉⠻⡿⠋⠀
 ⠀⠀⠀⠉⢿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠋⠘⢻⣿⣿⣿⣿⣷⣄⠀⠀⠀⠀⠀⠀⠀
 ⠀⠀⠀⠀⠀⠙⢿⣿⣿⣿⣿⡿⠋⠀⠀⠀⠀⢉⣿⣿⣿⣿⣿⡷⠀⠀⠀⠀⠀⠀
 ⠀⠀⠀⠀⠀⠀⠀⠉⢿⡿⠋⠀⠀⠀⠀⢀⣴⣿⣿⣿⣿⡿⠋⠀⠀⠀⠀⠀⠀⠀
 ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⣿⣿⣿⣿⡿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀
 ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢴⣿⣿⣿⣿⡿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢿⡿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 ***/

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

    public Servo plane;

    public Servo axon1;
    public Servo axon2;

    public Servo outake;
    public DcMotor intake;

    public static int outake_open = 1;
    public static int outake_close = 0;

    public static int plane_launch_pos = 0;
    public static int plane_armed_pos = 0;

    public static int axon_up_pos = 0;
    public static int axon_down_pos = 0;

    public static int slider_down = 0;

     public RobotUtils(HardwareMap hardwareMap){
      slider1 = hardwareMap.get(DcMotor.class, "slider1");
      slider2 = hardwareMap.get(DcMotor.class,"slider2");

      plane = hardwareMap.get(Servo.class, "plane");

      axon1 = hardwareMap.get(ServoImplEx.class, "brat1");
      axon2 = hardwareMap.get(ServoImplEx.class, "brat2");

      outake = hardwareMap.get(DcMotor.class, "outake");
      intake = hardwareMap.get(DcMotor.class, "intake");

      sensor1 = hardwareMap.get(RevColorSensorV3.class, "sensor1");

      slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      slider1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      slider2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      brat2.setDirection(Servo.Direction.REVERSE);

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
    public void bratUp(){
        brat1.setPosition(brat_sus);
        brat2.setPosition(brat_sus);
    }
    public void bratDown(){
        brat1.setPosition(brat_jos);
        brat2.setPosition(brat_jos);
    }
    public void cuva_closed(){gheara.setPosition(gheara_inchisa);}
    public void cuva_open(){gheara.setPosition(gheara_deschisa);}
    public void slider_up(){
       slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       goSliderToPosition(slider, 0.7);
    }
    public void slider_down(){
       slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        goSliderToPosition(10, 0.7);
    }
}
