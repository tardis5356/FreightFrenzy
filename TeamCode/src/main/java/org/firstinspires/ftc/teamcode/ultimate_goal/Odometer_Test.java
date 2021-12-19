package org.firstinspires.ftc.teamcode.ultimate_goal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Odometer_Test", group = "Linear Opmode")
@Disabled

public class Odometer_Test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor odNew = null;
    int offset = 0;
   // private DcMotor mR = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        odNew = hardwareMap.get(DcMotor.class, "odNew");
        //mR = hardwareMap.get(DcMotor.class, "mR");

        odNew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //mL.setDirection(DcMotor.Direction.FORWARD);
        //mR.setDirection(DcMotor.Direction.REVERSE);

        //mL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //mR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if(gamepad1.x){
                offset = odNew.getCurrentPosition();
            }

            telemetry.addData("odValue", (odNew.getCurrentPosition() - offset) /  425.68);
            telemetry.update();


        }
    }
}
