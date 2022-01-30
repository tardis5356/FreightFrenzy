package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Ginger_Sensor_Test", group = "Linear Opmode")

public class Ginger_Sensor_Test extends BaseClass_FF {

    @Override
    public void runOpMode() {

        defineComponentsGinger();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            gyroUpdate();
            updatePoseStrafe();

            telemetry.addData("gyro",gyroZ);
//            updatePoseStrafe();
//            double backRightDistanceDistance = rangeSensorBackRight.getDistance(DistanceUnit.CM);
//            double leftDistance = rangeSensorBackRight.getDistance(DistanceUnit.CM);
//            double leftDistance = rangeSensorBackRight.getDistance(DistanceUnit.CM);

//            double rightDistance = rangeSensorRight.getDistance(DistanceUnit.CM);

            telemetry.addData("left odometer wheel",mFL.getCurrentPosition());
            telemetry.addData("back odometer wheel",mFR.getCurrentPosition());
            telemetry.addData("right odometer wheel",-mBL.getCurrentPosition());
            telemetry.addData("x pos",pose.x);
            telemetry.addData("y pos",pose.y);
            telemetry.addData("currEnX", -(mFR.getCurrentPosition() - encoderXStart));
            telemetry.addData("currEnYLeft",(mFL.getCurrentPosition() - encoderYLeftStart));
            telemetry.addData("currEnYRight",-(mBL.getCurrentPosition() - encoderYRightStart));

            telemetry.addData(" left distance", "" + String.format("%.2f cm", Range.clip(rangeSensorLeft.getDistance(DistanceUnit.CM), 0, 200)));
            telemetry.addData(" front distance", "" + String.format("%.2f cm", Range.clip(rangeSensorFront.getDistance(DistanceUnit.CM), 0, 200)));
            telemetry.update();


        }
    }
}
