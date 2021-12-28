package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Fred_Sensor_Test", group = "Linear Opmode")

public class Fred_Sensor_Test extends BaseClass_FF {

    @Override
    public void runOpMode() {

        defineComponentsFred();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            gyroUpdate();

            telemetry.addData("gyro",gyroZ);
            updatePoseStrafe();
//            double leftDistance = rangeSensorLeft.getDistance(DistanceUnit.CM);
//            double rightDistance = rangeSensorRight.getDistance(DistanceUnit.CM);

//            telemetry.addData("left odometer wheel",mFL.getCurrentPosition());
//            telemetry.addData("back odometer wheel",mFR.getCurrentPosition());
//            telemetry.addData("right odometer wheel",mBL.getCurrentPosition());
//            telemetry.addData("x pos",pose.x);
//            telemetry.addData("y pos",pose.y);
            telemetry.addData("front right", mFR.getCurrentPosition());
            telemetry.addData("front left", mFL.getCurrentPosition());
            telemetry.addData("back right", mBR.getCurrentPosition());
            telemetry.addData("back left", mBL.getCurrentPosition());
            telemetry.addData(" back right distance", "" + String.format("%.2f in", Range.clip(rangeSensorBack.getDistance(DistanceUnit.INCH), 0, 200)));
            telemetry.addData(" left distance", "" + String.format("%.2f in", Range.clip(rangeSensorLeft.getDistance(DistanceUnit.INCH), 0, 200)));
            telemetry.addData(" right distance", "" + String.format("%.2f in", Range.clip(rangeSensorRight.getDistance(DistanceUnit.INCH), 0, 200)));
            telemetry.addData(" front distance", "" + String.format("%.2f in", Range.clip(rangeSensorFront.getDistance(DistanceUnit.INCH), 0, 200)));
            telemetry.addData("potentiometer angle", potentiometer.getVoltage());
            telemetry.update();




        }
    }
}
