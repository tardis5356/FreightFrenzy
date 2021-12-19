package org.firstinspires.ftc.teamcode.ultimate_goal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    @TeleOp
    @Disabled
    public class RangeSensorTest extends BaseClass_TestBed {
        //DistanceSensor distance;
        //DcMotor motor;

        @Override
        public void runOpMode() {
            defineComponents();

            // Get the distance sensor and motor from hardwareMap
            //distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
            //motor = hardwareMap.get(DcMotor.class, "Motor");

            // Loop while the Op Mode is running
            waitForStart();
            while (opModeIsActive()) {
                // If the distance in centimeters is greater than 100, set the power to 0.3
                if (rangeSensorLeft.getDistance(DistanceUnit.CM) > 40) {
                    drive(-0.6, 0, 0);
                    // motor.setPower(0.3);

                } else if (rangeSensorLeft.getDistance(DistanceUnit.CM) < 20) {
                    drive(0.6, 0, 0);


                } else {  // Otherwise, stop the motor\
                    stopDriveTrain();
                    //motor.setPower(0);
                }
            }
        }
    }


