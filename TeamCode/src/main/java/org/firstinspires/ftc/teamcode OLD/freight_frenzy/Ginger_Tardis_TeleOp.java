package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Ginger_Tardis_TeleOp", group = "Linear Opmode")
@Disabled

public class Ginger_Tardis_TeleOp extends BaseClass_FF {    // LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor mFL = null;
//   private DcMotor mFR = null;
// changed motor names in entire program to mFL and mFR from mL and mR

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        defineComponentsGinger();
        double powerMultiplier = 0.6;
        boolean previousBState = false;
        boolean motorPowerFast = false;
        boolean wirelessConnected = true;
        //double sWHPosition = sWH.getPosition();
       // double sWVPosition = sV.getPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            /*
            telemetry.addData("gyro", "" + String.format("%.2f deg", gyroZ));
            telemetry.addData("current position x", "" + String.format("%.2f in.", pose.x));
            telemetry.addData("current position y", "" + String.format("%.2f in.", pose.y));
            telemetry.addData("Motor power fast", motorPowerFast);
            telemetry.addData("Wireless connection connected", wirelessConnected);
            telemetry.addData("Wrist servo horizontal position", sWH.getPosition());
            telemetry.addData("Wrist servo vertical position", sWV.getPosition());
            //telemetry.addData("Intake left power", crsIL.getPower());
            //telemetry.addData("Intake right power", crsIR.getPower());
            telemetry.update();

            //Update global sensor values
             */
            updatePoseStrafe();
            gyroUpdate();

            //Gamepad 1 Variables
            // waitForStart();
            runtime.reset();
            double leftY1 = gamepad1.left_stick_y * powerMultiplier;//drive straight
            double rightX1 = -(gamepad1.right_stick_x) * powerMultiplier;//drive turn
            double leftX1 = -(gamepad1.left_stick_x) * powerMultiplier;//drive strafe
            double rightY1 = -(gamepad1.right_stick_y);
            boolean bButton = (gamepad1.b);//drive speed toggle
            boolean yButton = (gamepad1.y);
            double rightTrigger = (gamepad1.right_trigger);
            double leftTrigger = -(gamepad1.left_trigger);
            boolean aButton = (gamepad1.a);
            boolean xButton = (gamepad1.x);
            //Gamepad 2 Variables
            double rightTrigger2 = (gamepad2.right_trigger);//sWV servo
            double rightY2 = (gamepad2.right_stick_y);//arm top motor
            double leftY2 = (gamepad2.left_stick_y);//arm bottom motor
            double leftTrigger2 = (gamepad2.left_trigger);//sWV servo
            boolean leftBumper = (gamepad2.left_bumper);//sWH servo
            boolean rightBumper = (gamepad2.right_bumper);//sWH servo
            boolean bButton2 = (gamepad2.b);
            boolean aButton2 = (gamepad2.a);
            boolean xButton2 = (gamepad2.x);
            boolean yButton2 = (gamepad2.y);

            //telemetry.addData("rangeSensorBackLeft i2c addy", rangeSensorBackLeft.getDeviceName());
            telemetry.update();
            // telemetry.addData("LeftX1 position", leftX1);
            // telemetry.addData("RightX1 position", rightX1);
            // telemetry.addData("LeftY2 position", leftY2);
            // telemetry.addData("RightY2 position", rightY2);
            // telemetry.addData("Right trigger position", rightTrigger2);


            //drives robot
            drive(leftY1, -rightX1, -leftX1);

            //moves bottom and top arm joints
            mE.setPower(-leftY2); //also works for mF on Toby bot
            mU.setPower(rightY2);

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

            //control wrist left-right motion
            //pressed = 1, released = 0
            /*if (rightBumper) { //&& sWHPosition < 1)
                sWHPosition += .008;
            } else if (leftBumper) {// && sWHPosition > 0) {
                sWHPosition -= .008;
            }
            sWH.setPosition(Range.clip(sWHPosition, 0, 1));
            sWHPosition = sWH.getPosition();*/

            //controls wrist up-down motion
            /*if ((rightTrigger2 >= 0.99)) {//&& (sWVPosition < 1))
                sWVPosition += .008;
            } else if ((leftTrigger2 >= 0.99)) { //&& (sWVPosition > 0))
                sWVPosition -= .008;
            }
            sV.setPosition(Range.clip(sWVPosition, 0, 1));
            sWVPosition = sV.getPosition();*/

            /*
            //controls intake
            if (xButton2) {

                //sucks blocks in
                // sI.setPower(-0.5);
                crsIL.setPower(-0.5);
                crsIR.setPower(0.5);
            } else if (yButton2) {

                //spits blocks out
                // sI.setPower(0.5);
                crsIL.setPower(0.5);
                crsIR.setPower(-0.5);

            } else if (!xButton2 && !yButton2) {

                //if both buttons are not pressed, power is off
                // sI.setPower(0);
                crsIL.setPower(0);
                crsIR.setPower(0);
            }
            */

            //controls intake
            if (xButton2) {

                //sucks blocks in
                // sI.setPower(-0.5);
                crsIL.setPower(-0.5);
                crsIR.setPower(0.5);
            } else if (yButton2) {

                //spits blocks out
                // sI.setPower(0.5);
                crsIL.setPower(0.5);
                crsIR.setPower(-0.5);

            } else if (!xButton2 && !yButton2) {

                //if both buttons are not pressed, power is off
                // sI.setPower(0);
                crsIL.setPower(0);
                crsIR.setPower(0);
            }

            if(aButton){
                //spinner for carousel
                sSR.setPower(1);
                sSL.setPower(-1);
            } else {
                sSR.setPower(0);
                sSL.setPower(0);
            }
        }
            //else
//
//                //default: power off
//                crsIL.setPower(0);
//                crsIR.setPower(0);
//            }

//            if(xButton){
//                sSpinner.setPower(1);
//            }else{
//                sSpinner.setPower(0);
//            }

            //previousBState = bButton;


        }


    }

