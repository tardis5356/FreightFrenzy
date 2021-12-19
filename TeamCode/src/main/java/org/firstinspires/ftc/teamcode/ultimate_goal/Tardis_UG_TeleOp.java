package org.firstinspires.ftc.teamcode.ultimate_goal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Tardis_UG_TeleOp", group = "Linear Opmode")
@Disabled

public class Tardis_UG_TeleOp extends BaseClassTB1 {    // LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor mFL = null;
//   private DcMotor mFR = null;
// changed motor names in entire program to mFL and mFR from mL and mR

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        defineComponents();
        double wristPosition = 1;
        boolean shooterMotorIdle = true;
        boolean previousB2State = false;
        boolean motorPowerFast = false;
        double powerMultiplier = 0.5;
        boolean previousBState = false;
        boolean amIFiring = false;
        double george = 0;
        double armStartPosition = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        sW.setPosition(1);

        waitForStart();

        while (opModeIsActive()) {
            george = potentiometer.getVoltage();
            telemetry.addData("drivetrain power multiplier", powerMultiplier);
            telemetry.addData("gyro", "" + String.format("%.2f deg", gyroZ));
            telemetry.addData("arm position", mA.getCurrentPosition());
            telemetry.addData("wrist position", "" + String.format("%.2f", wristPosition));
            telemetry.addData("potentiometer angle", getElevAngle(george));
            telemetry.addData("potentiometer voltage", george);
            telemetry.addData("gripper position", sG.getPosition());
            telemetry.addData("fire ring", sP.getPosition());
            telemetry.addData("current position x", "" + String.format("%.2f in.", pose.x));
            telemetry.addData("current position y", "" + String.format("%.2f in.", pose.y));
            telemetry.addData("front limit", frontLimit.isPressed());
            telemetry.addData("back limit", backLimit.isPressed());
            telemetry.update();
            //Update global sensor values
            updatePoseStrafe();
            gyroUpdate();

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
            boolean leftBumper = (gamepad2.left_bumper);
            boolean rightBumper = (gamepad2.right_bumper);
            boolean bButton2 = (gamepad2.b);
            boolean aButton2 = (gamepad2.a);
            boolean bButton = (gamepad1.b);
            boolean yButton = (gamepad1.y);
            boolean xButton2 = (gamepad2.x);


            telemetry.addData("leftY2", leftY2);

            //sets shooting position
            if (yButton) {
                goToAngle(30);
                //mS.setPower(1);
                moveToPose(32, -12, 0, 50);
            } else {
                drive(leftY1, leftX1, rightX1);
            }



            //moves arm to position for wobble pick up
            if(xButton2) {
                sW.setPosition(0.4);
                if(frontLimit.isPressed()) {
                    armStartPosition = mA.getCurrentPosition();
                    mA.setPower(0);
                    wristPosition = 0.36;
                }
                else if (!frontLimit.isPressed()) {
                    mA.setPower(-1);
                }
            }

            //raises arm to pick up wobble
            if(aButton2) {
                double armMotion = 2000;
                double armStopPosition = armStartPosition + armMotion; //calculates end arm position

                if (mA.getCurrentPosition() >= armStopPosition) {
                    mA.setPower(0);
                } else {
                    mA.setPower(1);
                }


                wristPosition = 0.3;
            }

            //arm
            if(!xButton2 && !aButton2) {
                telemetry.addData("arm joystick:", rightY2);
                if(!backLimit.isPressed()){
                    mA.setPower(rightY2);
                }else if(backLimit.isPressed() && leftY2 <= 0){
                    mA.setPower(rightY2);
                }
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


            //potentiometer/elevator set angle
            if(!yButton && getElevAngle(george) > 2){
                mE.setPower(leftY2);
            }else if(getElevAngle(george) < 2 && leftY2 < 1){
                mE.setPower(leftY2);
            }

            //intake - negative is pulling in, positive is spitting out
            // goToAngle sets elevator to level that+ is optimal for intake of rings
            if ((rightTrigger != 0) && (leftTrigger != 0)) {
                mI.setPower(0);
            } else if ((leftTrigger != 0) && (sLower.getPosition() == 1)) {
                mI.setPower(-1);
                goToAngle(-4);
            } else if ((rightTrigger != 0) && (sLower.getPosition() == 1)) {
                mI.setPower(1);
                goToAngle(-4);
            } else {
                mI.setPower(0);
            }


            //code to fire a ring

//            if (leftTrigger2 != 0) {
//                amIFiring = true;
//            }
//
//            if (sP.getPosition() >= 0.98) {
//                amIFiring = false;
//            }
//
//            if (amIFiring) {
//                sP.setPosition(1);
//            } else {
//                sP.setPosition(0);
//            }


            //fires ring
            if (leftTrigger2 != 0) {
                sP.setPosition(1);//shooting
            } else {
                sP.setPosition(0);//not shooting
            }

            //toggles shooting motor on/off
            // this should only run once each time b button is pressed
            if (bButton2 != previousB2State && bButton2) {
                if (shooterMotorIdle) {
                    shooterMotorIdle = false;
                    mS.setPower(1);
                } else {
                    shooterMotorIdle = true;
                    mS.setPower(0.25);
                }
            }

            //controls wrist
            if(!xButton2) {
                if (rightBumper && wristPosition < 1) {
                    wristPosition += .008;
                } else if (leftBumper && wristPosition > 0) {
                    wristPosition -= .008;
                }
                sW.setPosition(Range.clip(wristPosition, 0, 1));

            }

            //controls gripper (pressed = closed, released = open)
            if (rightTrigger2 != 0 && !aButton2){
                sG.setPosition(0.25); //sets gripper to open         // 0.8, 0.35
            } else {
                if(!aButton2) {
                    sG.setPosition(0.08); //sets gripper to closed   // 0.5, 0.18
                }
            }

            //controls release of intake ramp
            if (xButton) {
                sLower.setPosition(1); //released
            }
            /*else{
                sLower.setPosition(0); //default, secured
            }*/


            previousBState = bButton;
            previousB2State = bButton2;


        }
    }


}

