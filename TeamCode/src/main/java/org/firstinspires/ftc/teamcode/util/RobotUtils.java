package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotUtils {
    DcMotor slider1;
    DcMotor slider2;
    DcMotor tureta;
    Servo servo1;
    Servo servo2;
    Servo intake;
    RevColorSensorV3 sensor;
    int poz_ridicat = 0;
    int poz_jos = 15;
    int poz_extins = 0;
    int poz_retras = 0;
    int poz_intake_open = 0;
    int poz_intake_close = 0;
    int distanta_de_detectat = 10;
    public RobotUtils(HardwareMap hardwareMap){

        ///Declararea fizica a sistemelor de pe robot
        slider1 = hardwareMap.get(DcMotor.class, "slider1");
        slider2 = hardwareMap.get(DcMotor.class, "slider2");
        tureta = hardwareMap.get(DcMotor.class, "tureta");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        intake = hardwareMap.get(Servo.class, "intake");
        sensor = hardwareMap.get(RevColorSensorV3.class, "sensor");

        ///Selectarea modului de control a motoarelor
        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tureta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ///Reverse pentru pozitile servo-ului
        servo2.setDirection(Servo.Direction.REVERSE);
    }

    public void goSliderToPosition(int position, double power) {
        // Ensure that power is positive.
        double absPower = Math.abs(power);

        // Get the current position of the slider.
        int currentPos = slider1.getCurrentPosition();

        // Set the target position of both slider motors.
        slider1.setTargetPosition(position);
        slider2.setTargetPosition(position);

        // Set the run mode of both slider motors to RUN_TO_POSITION.
        slider1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    public void extend(){
        servo1.setPosition(poz_extins);
        servo2.setPosition(poz_extins);
    }

    public void retract(){
        servo1.setPosition(poz_retras);
        servo2.setPosition(poz_retras);
    }

    public void open_intake(){
        intake.setPosition(poz_intake_open);
    }

    public void close_intake(){
        intake.setPosition(poz_intake_close);
    }

    public boolean HasDetected(){
        if(sensor.getDistance(DistanceUnit.MM)<=distanta_de_detectat)
            return true;
        return false;
    }
}
