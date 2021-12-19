package org.firstinspires.ftc.teamcode.ultimate_goal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Sensor_Test", group = "Linear Opmode")
@Disabled

public class Sensor_Test extends BaseClassTB1 {

    @Override
    public void runOpMode() {

        defineComponents();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            gyroUpdate();

            telemetry.addData("gyro",gyroZ);
            updatePoseStrafe();

            telemetry.addData("left odometer wheel",mFL.getCurrentPosition());
            telemetry.addData("back odometer wheel",mFR.getCurrentPosition());
            telemetry.addData("right odometer wheel",mBL.getCurrentPosition());
            telemetry.addData("x pos",pose.x);
            telemetry.addData("y pos",pose.y);
            telemetry.addData("", "");
            telemetry.update();


        }
    }
}
