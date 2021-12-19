package org.firstinspires.ftc.teamcode.ultimate_goal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class BaseClass_DemoRobot extends LinearOpMode {
    DcMotor mL;
    DcMotor mR;
    DcMotor mA;

    Servo sG;
    Servo sGA;


public void defineComponents() {
    mL = hardwareMap.dcMotor.get("mL");
    mR = hardwareMap.dcMotor.get("mR");
    mA = hardwareMap.dcMotor.get("mA");

    sG = hardwareMap.servo.get("sG");
    sGA = hardwareMap.servo.get("sGA");
    }
}