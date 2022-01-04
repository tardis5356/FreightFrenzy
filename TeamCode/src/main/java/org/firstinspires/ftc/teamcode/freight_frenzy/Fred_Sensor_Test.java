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
            readDistanceSensors();

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
            telemetry.addData("gyro", "" + String.format("%.2f deg", gyroZ));
            telemetry.addData("back distance (in)", "" + String.format("%.2f", backDistance));
            telemetry.addData("back distance filtered (in)", "" + String.format("%.2f", backDistanceFiltered));
            telemetry.addData("front distance (in)", "" + String.format("%.2f", frontDistance));
            telemetry.addData("front distance filtered (in)", "" + String.format("%.2f", frontDistanceFiltered));
            telemetry.addData("left distance (in)", "" + String.format("%.2f", leftDistance));
            telemetry.addData("left distance filtered (in)", "" + String.format("%.2f", leftDistanceFiltered));
            telemetry.addData("right distance (in)", "" + String.format("%.2f", rightDistance));
            telemetry.addData("right distance filtered (in)", "" + String.format("%.2f", rightDistanceFiltered));
            telemetry.addData("potentiometer angle", potentiometer.getVoltage());
            telemetry.addData("extension position", mE.getCurrentPosition());
            telemetry.addData("arm limit", lAB.isPressed());
            telemetry.update();




        }
    }
}
