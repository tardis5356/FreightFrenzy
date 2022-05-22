package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Tardis_Omni_TeleOp", group = "Linear Opmode")
//@Disabled

public class Tardis_Omni_TeleOp extends BaseClass_FF {    // LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor mFL = null;
//   private DcMotor mFR = null;
// changed motor names in entire program to mFL and mFR from mL and mR

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        defineComponentsOmni();
        boolean motorPowerFast = false;
        double powerMultiplier = 0.5;
        boolean previousBState = false;


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            //george = potentiometer.getVoltage();
            telemetry.addData("right odometer wheel", mBL.getCurrentPosition());
            telemetry.addData("left odometer wheel", mFR.getCurrentPosition());
            telemetry.addData("back odometer wheel", mFL.getCurrentPosition());
            telemetry.addData("drivetrain power multiplier", powerMultiplier);
            telemetry.update();
            //Update global sensor values
//            updatePoseStrafe();
            //gyroUpdate();

            //Gamepad 1 Variables
            waitForStart();
            runtime.reset();
            double leftY1 = gamepad1.left_stick_y * powerMultiplier;
            double rightX1 = -(gamepad1.right_stick_x) / 2.5;
            double leftX1 = -(gamepad1.left_stick_x) * powerMultiplier;
            double rightY1 = -(gamepad1.right_stick_y);
            double rightTrigger = (gamepad1.right_trigger);
            double leftTrigger = -(gamepad1.left_trigger);
            boolean xButton = (gamepad1.x);
            //Gamepad 2 Variables
            double rightTrigger2 = (gamepad2.right_trigger);
            double rightY2 = (gamepad2.right_stick_y);
            double leftY2 = (gamepad2.left_stick_y);
            double leftTrigger2 = (gamepad2.left_trigger);
            boolean leftBumper2 = (gamepad2.left_bumper);
            boolean rightBumper2 = (gamepad2.right_bumper);
            boolean bButton2 = (gamepad2.b);
            boolean aButton2 = (gamepad2.a);
            boolean bButton = (gamepad1.b);
            boolean yButton2 = (gamepad1.y);
            boolean xButton2 = (gamepad2.x);


            telemetry.addData("leftY2", leftY2);

            drive(leftY1, leftX1, rightX1);

            mTGIArm.setPower(leftY2);

            if (rightTrigger2 != 0) {
                sG.setPosition(0);
            } else {
                sG.setPosition(1);
            }

            //changes drive speed
            if (bButton != previousBState && bButton) {
                if (motorPowerFast) {
                    motorPowerFast = false;
                    powerMultiplier = 1;
                } else {
                    motorPowerFast = true;
                    powerMultiplier = 0.5;
                }
            }

            previousBState = bButton;
//            previousB2State = bButton2;


        }
    }


}

