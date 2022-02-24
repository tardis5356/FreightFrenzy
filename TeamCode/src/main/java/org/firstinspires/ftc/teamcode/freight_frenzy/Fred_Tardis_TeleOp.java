package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

@TeleOp(name = "Fred_Tardis_TeleOp", group = "Linear Opmode")

public class Fred_Tardis_TeleOp extends BaseClass_FF {    // LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public enum ArmState {
        INIT,
        BACK_INTAKE,
        BACK_DELIVERY,
        CAP_INTAKE,
        CAP_DELIVERY,
        NEUTRAL,
        FREE
    }

    ;
    ArmState armState = ArmState.NEUTRAL;

    public enum IntakeState {
        TEN_SEC_INTAKE,
        INDEFININTE_INTAKE,
        FIVE_SEC_DELIVERY,
        INDEFININTE_DELIVERY,
        TOGGLE,
        OFF
    }

    IntakeState intakeState = IntakeState.OFF;

    ElapsedTime intakeTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        defineComponentsFred();
        double powerMultiplier = 0.6;
        boolean previousBState = false;
        boolean motorPowerFast = false;
        boolean extensionReset = false;
        double sVPosition = sV.getPosition();
        armLimitOffset = 0;
        boolean intakeStateSet = false;

        double power = 0;
        double time = 0;
        int timeCount = 0;
        double timeShift = 9;
        double[] powers = {0.5, 0.6, 0.6, 0.6, 1};

//        //arm automation presets
        //01-02-21
        double neutralUpright = 1.08;
        double initUpright = neutralUpright + 1.8;
        double capIntakeUpright = neutralUpright + 2.1;
        double capDeliveryUpright = neutralUpright + 0.6;
        double backDeliveryUpright = neutralUpright - 0.21;
        double backIntakeUpright = neutralUpright - 0.9;

        double neutralWrist = 0.67;
        double initWrist = neutralWrist - 0.6;
        double capIntakeWrist = neutralWrist - 0.09;
        double capDeliveryWrist = neutralWrist - 0.35;
        double backDeliveryWrist = neutralWrist + 0.25;
        double backIntakeWrist = neutralWrist + 0.2;

        double neutralExtension = 100;
        double initExtension = 100;
        double capIntakeExtension = 300;
        double capDeliveryExtension = 900;
        double backDeliveryExtension = 500;
        double backIntakeExtension = 800;

        int extensionTolerance = 50;

        intakeTimer.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updatePoseStrafe();
            gyroUpdate();

            //Gamepad 1 Variables
            runtime.reset();

            double leftY1 = gamepad1.left_stick_y * powerMultiplier;//drive forward
            double rightX1 = -(gamepad1.right_stick_x) * powerMultiplier;//drive rotate
            double leftX1 = -(gamepad1.left_stick_x) * powerMultiplier;//drive strafe
            double rightY1 = -(gamepad1.right_stick_y);//free
            double rightTrigger = (gamepad1.right_trigger);//capstone arm
            double leftTrigger = (gamepad1.left_trigger);//capstone arm
            boolean rightBumper = (gamepad1.right_bumper);//capstone gripper
            boolean xButton = (gamepad1.x);//reverse spinner direction
            boolean bButton = (gamepad1.b);//turbo drive
            boolean yButton = (gamepad1.y);//intake reverse
            boolean aButton = (gamepad1.a);//spinner

            //Gamepad 2 Variables
            double rightTrigger2 = (gamepad2.right_trigger);//wrist up
            double rightY2 = (gamepad2.right_stick_y);//arm angle
            double leftY2 = (gamepad2.left_stick_y);//arm extend
            double leftTrigger2 = (gamepad2.left_trigger);//wrist down
            boolean leftBumper2 = (gamepad2.left_bumper);//free
            boolean rightBumper2 = (gamepad2.right_bumper);//free
            boolean bButton2 = (gamepad2.b);//sets arm for intake
            boolean aButton2 = (gamepad2.a);//sets arm for delivery
            boolean yButton2 = (gamepad2.y);//sets wrist for intake
            boolean xButton2 = (gamepad2.x);//sets wrist for delivery
            boolean dpadUp2 = (gamepad2.dpad_up);
            boolean dpadDown2 = (gamepad2.dpad_down);
            boolean dpadLeft2 = (gamepad2.dpad_left);
            boolean dpadRight2 = (gamepad2.dpad_right);

            telemetry.addData("gyro", "" + String.format("%.2f deg", gyroZ));
            telemetry.addData("Motor power fast", motorPowerFast);
            telemetry.addData("Wrist angle", sVPosition);
            telemetry.addData("potentiometer voltage", potentiometer.getVoltage());
            telemetry.addData("extension position", mE.getCurrentPosition());
            telemetry.addData("arm limit", lAB.isPressed());
            telemetry.addData("arm limit offset", armLimitOffset);
            telemetry.addData("intake state", intakeState);
            telemetry.addData("arm state", armState);
            telemetry.addData("extensionReset", extensionReset);
            telemetry.addData("mE mode", mE.getMode());
            telemetry.addData("mE current position", mE.getCurrentPosition());
            telemetry.addData("intakeState", intakeState);
            telemetry.addData("intakeTimer", intakeTimer);
            telemetry.update();

            //drives robot
            drive(leftY1, leftX1, rightX1);

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

            //deploy odometers
            if (xButton) {
                lowerOdometerServos();
            } else {
                raiseOdometerServos();
            }

            //controls wrist up-down motion
            if ((rightBumper2)) {//&& (sWVPosition < 1))
                sVPosition += .01;
            } else if ((leftBumper2)) { //&& (sWVPosition > 0))
                sVPosition -= .01;
            }
            sV.setPosition(Range.clip(sVPosition, 0.01, 1));
            sVPosition = sV.getPosition();

            if (!extensionReset) {
                mE.setPower(-1);
                if (lAB.isPressed()) {
                    mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extensionReset = true;
                }
            } else if (extensionReset) {
                mE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            //arm
            if (yButton2) {// neutral position
                armState = ArmState.NEUTRAL;
            } else if (bButton2) {// backDelivery position
                armState = ArmState.BACK_DELIVERY;
            } else if (xButton2) {// backIntake position
                armState = ArmState.BACK_INTAKE;
            } else if (aButton2) {// capIntake position
                armState = ArmState.CAP_INTAKE;
            } else if (dpadRight2) {// capDelivery
                armState = ArmState.CAP_DELIVERY;
            } else if (dpadLeft2) {// init
                armState = ArmState.INIT;
            }

            if (gamepad2.y || gamepad1.b || gamepad2.a || gamepad2.x || gamepad2.dpad_right || gamepad2.dpad_left) {
                intakeStateSet = false;
            }

            if(extensionReset) {
                switch (armState) {
                    case NEUTRAL:
                        armToPosPID(neutralUpright);
                        extensionToPos(neutralExtension, extensionTolerance);
                        sVPosition = neutralWrist;
                        if (!intakeStateSet) {
                            intakeState = IntakeState.TOGGLE;
                            intakeStateSet = true;
                        }
                        break;
                    case BACK_INTAKE:
                        armToPosPID(backIntakeUpright);
                        extensionToPos(backIntakeExtension, extensionTolerance);
                        sVPosition = backIntakeWrist;
                        if (!intakeStateSet) {
                            intakeState = IntakeState.TEN_SEC_INTAKE;
                            intakeStateSet = true;
                        }
                        break;
                    case BACK_DELIVERY:
                        armToPosPID(backDeliveryUpright);
                        extensionToPos(backDeliveryExtension, extensionTolerance);
                        sVPosition = backDeliveryWrist;
                        if (!intakeStateSet) {
                            intakeState = IntakeState.TOGGLE;
                            intakeStateSet = true;
                        }
                        break;
                    case CAP_INTAKE:
                        armToPosPID(capIntakeUpright);
                        extensionToPos(capIntakeExtension, extensionTolerance);
                        sVPosition = capIntakeWrist;
                        if (!intakeStateSet) {
                            intakeState = IntakeState.TEN_SEC_INTAKE;
                            intakeStateSet = true;
                        }
                        break;
                    case CAP_DELIVERY:
                        armToPosPID(capDeliveryUpright);
                        extensionToPos(capDeliveryExtension, extensionTolerance);
                        sVPosition = capDeliveryWrist;
                        if (!intakeStateSet) {
                            intakeState = IntakeState.TOGGLE;
                            intakeStateSet = true;
                        }
                        break;
                    case INIT:
                        armToPosPID(initUpright);
                        extensionToPos(initExtension, extensionTolerance);
                        sVPosition = initWrist;
                        if (!intakeStateSet) {
                            intakeState = IntakeState.TOGGLE;
                            intakeStateSet = true;
                        }
                        break;
                    case FREE:
                        //sets arm extension and arm upright motion
                        mE.setPower(-leftY2); //also works for mF on Toby bot
                        mU.setPower(rightY2 * 0.8);//0.8 power multiplier
                        break;
                }
            }

            // intake (in postitive, out negative)
            switch (intakeState) {
                case TEN_SEC_INTAKE:
                    sI.setPower(-1);
                    if (intakeTimer.seconds() > 10) {
                        intakeState = IntakeState.TOGGLE;
                    }
                    break;
                case INDEFININTE_INTAKE:
                    sI.setPower(-1);
                    break;
                case FIVE_SEC_DELIVERY:
                    sI.setPower(1);
                    if (intakeTimer.seconds() > 5) {
                        intakeState = IntakeState.TOGGLE;
                    }
                    break;
                case INDEFININTE_DELIVERY:
                    sI.setPower(1);
                    break;
                case TOGGLE:
                    intakeTimer.reset();
                    if (leftTrigger2 == 1 && rightTrigger2 == 0) {
                        //sucks elements in
                        sI.setPower(1);
                    } else if (rightTrigger2 == 1 && leftTrigger2 == 0) {
                        //spits elements out
                        sI.setPower(-1);
                    } else {
                        sI.setPower(0);
                    }
                    break;
            }

            //finite state overrides
            if (dpadUp2) { // no control
                armState = ArmState.FREE;
            } else if (dpadDown2) {
                intakeState = IntakeState.TOGGLE;
            }

            //auto arm overrides
            if ((rightY2 != 0) || (leftY2 != 0) || (rightBumper2) || (leftBumper2)) {
                armState = ArmState.FREE;
            }
            //auto intake overrides
            if ((rightTrigger2 != 0) || (leftTrigger2 != 0)) {
                intakeState = IntakeState.TOGGLE;
            }

            //carousel
            if (rightTrigger > 0.5 || leftTrigger > 0.5) {
                if (timeCount + 1 < powers.length) {
                    time += 1;
                    if (time > timeShift) {
                        timeCount++;
                        time = 0;
                    }
                }
                if (power < powers[timeCount]) {
                    power += 0.1;
                }
                if (rightTrigger > 0.5) {
                    mSL.setPower(power);
                    mSR.setPower(1);
                } else {
                    mSL.setPower(-power);
                    mSR.setPower(-1);
                }
            } else {
                mSL.setPower(0);
                mSR.setPower(0);
                power = 0;
                timeCount = 0;
                time = 0;
            }

            previousBState = bButton;
        }


    }


}
