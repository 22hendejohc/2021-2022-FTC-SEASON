package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

public class ObjectGrab {
    DcMotor arm;
    Servo wrist;
    Servo pinchLeft;
    Servo pinchRight;
    ColorSensor colorSensor;

    public void in() throws InterruptedException {
        arm.setPower(1);
        sleep(1000);
    }

    public void out() {

    }
    
}
