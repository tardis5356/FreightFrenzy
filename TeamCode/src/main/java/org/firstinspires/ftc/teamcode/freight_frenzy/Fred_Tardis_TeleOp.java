package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
        boolean previousRightBumperState = false;
        boolean previousX2State = false;
        boolean grippingCapstone = false;
        boolean motorPowerFast = false;
        boolean intaking = false;
        boolean wirelessConnected = true;
        double sVPosition = sV.getPosition();
//        double sCUPosition = sCU.getPosition();
        //used later to determine intake and delivery points for the arm
        double armLevelReading = mU.getCurrentPosition();
        double extensionPosition = mE.getCurrentPosition();
        armLimitOffset = 0;

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
        double deliveryUpright = neutralUpright - 0.21;
        double backIntakeUpright = neutralUpright - 0.9;

        double neutralWrist = 0.67;
        double initWrist = neutralWrist - 0.6;
        double capIntakeWrist = neutralWrist - 0.17;
        double capDeliveryWrist = neutralWrist - 0.35;
        double deliveryWrist = neutralWrist + 0.25;
        double backIntakeWrist = neutralWrist + 0.2;

        double neutralExtension = 100;
        double initExtension = 100;
        double capIntakeExtension = 400;
        double capDeliveryExtension = 2400;
        double deliveryExtension = 600;
        double backIntakeExtension = 1500;

        double toleranceU = 0;
        double toleranceE = 0;

        intakeTimer.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updatePoseStrafe();
            gyroUpdate();

            //Gamepad 1 Variables
            // waitForStart();
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

            telemetry.addData("prevRBumper", previousRightBumperState);
//            telemetry.addData("capstone gripper servo", sCG.getPosition());
            telemetry.addData("leftTrigger2", leftTrigger2);
            telemetry.addData("rightTrigger2", rightTrigger2);
            telemetry.addData("LeftY1 position", leftY1);
            telemetry.addData("LeftX1 position", leftX1);
            telemetry.addData("RightX1 position", rightX1);
            telemetry.addData("gyro", "" + String.format("%.2f deg", gyroZ));
            telemetry.addData("Motor power fast", motorPowerFast);
            telemetry.addData("Wireless connection connected", wirelessConnected);
            telemetry.addData("Left Bumper 2", leftBumper2);
            telemetry.addData("Right Bumper 2", rightBumper2);
            telemetry.addData("aButton is pressed", aButton);
            telemetry.addData("Wrist angle", sVPosition);
            telemetry.addData("Arm angle", mU.getCurrentPosition());
            telemetry.addData("potentiometer voltage", potentiometer.getVoltage());
            telemetry.addData("extension position", mE.getCurrentPosition());
            telemetry.addData("arm limit", lAB.isPressed());
            telemetry.addData("arm limit offset", armLimitOffset);
            telemetry.addData("power", power);
            telemetry.addData("time", time);
            telemetry.addData("timeCount", timeCount);
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

            //sets arm limit based on limit switch
            if (!bButton2 && !aButton2) {
                if (lAB.isPressed()) {
                    armLimitOffset = mE.getCurrentPosition();
                }
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

            switch (armState) {
                case NEUTRAL:
                    toleranceU = 0.3;
                    toleranceE = 100;

                    if (armLimitOffset != 0) {
                        if (Math.abs(potentiometer.getVoltage() - neutralUpright) < 0.5) { //prevent upright motor from being overstressed
                            if ((mE.getCurrentPosition() - armLimitOffset - neutralExtension) >= toleranceE) {
                                mE.setPower(-scaleShift((mE.getCurrentPosition() - armLimitOffset - neutralExtension), 0, 600, 1, 0, 1) - 0.2);//-1
                                telemetry.addData("-mE", scaleShift((mE.getCurrentPosition() - armLimitOffset - neutralExtension), 0, 600, 1, 0, 1) + 0.2);
                            } else if ((mE.getCurrentPosition() - armLimitOffset - neutralExtension) < toleranceE) {
                                mE.setPower(-scaleShift((mE.getCurrentPosition() - armLimitOffset - neutralExtension), 0, 600, 1, 0, 1));//1
                                telemetry.addData("+mE", scaleShift((mE.getCurrentPosition() - armLimitOffset - neutralExtension), 0, 600, 1, 0, 1) + 0.2);
                            } else {
                                mE.setPower(0);
                            }
                        }
                    } else {
                        mE.setPower(-0.5);
                        if (lAB.isPressed()) {
                            armLimitOffset = mE.getCurrentPosition();
                        }
                    }

                    if ((potentiometer.getVoltage() - neutralUpright) >= toleranceU) {
                        mU.setPower(-scaleShift((potentiometer.getVoltage() - neutralUpright), 0.3, 4, 1, 0, 4) - 0.4);
                        telemetry.addData("neutralnegative", (((((potentiometer.getVoltage() - neutralUpright) - 0) * (1 - 0)) / (4 - 0)) + 0));

                    } else if ((potentiometer.getVoltage() - neutralUpright) < toleranceU) {
                        mU.setPower(-1 + scaleShift((potentiometer.getVoltage() - neutralUpright), 0, 4, 0, 1, 8) + 0.4);
                        telemetry.addData("neutralpositive", (-1 + scaleShift((potentiometer.getVoltage() - neutralUpright), 0, 4, 0, 1, 8) + 0.4));
                    } else {
                        mU.setPower(0);
                        intakeState = IntakeState.OFF;
                        armState = ArmState.FREE;
                    }

                    sVPosition = neutralWrist;
                    break;
                case BACK_INTAKE:
                    toleranceU = 0.4;
                    toleranceE = 100;

                    if (armLimitOffset != 0) {
                        if (Math.abs(potentiometer.getVoltage() - backIntakeUpright) < 0.5) { //prevent upright motor from being overstressed
                            if ((mE.getCurrentPosition() - armLimitOffset - backIntakeExtension) >= toleranceE) {
                                mE.setPower(-scaleShift((mE.getCurrentPosition() - armLimitOffset - backIntakeExtension), 0, 600, 1, 0, 1) - 0.2);//-1
                                telemetry.addData("-mE", scaleShift((mE.getCurrentPosition() - armLimitOffset - backIntakeExtension), 0, 600, 1, 0, 1) + 0.2);
                            } else if ((mE.getCurrentPosition() - armLimitOffset - backIntakeExtension) < toleranceE) {
                                mE.setPower(-scaleShift((mE.getCurrentPosition() - armLimitOffset - backIntakeExtension), 0, 600, 1, 0, 1));//1
                                telemetry.addData("+mE", scaleShift((mE.getCurrentPosition() - armLimitOffset - backIntakeExtension), 0, 600, 1, 0, 1) + 0.2);
                            } else {
                                mE.setPower(0);
                            }
                        }
                    } else {
                        mE.setPower(-0.5);
                        if (lAB.isPressed()) {
                            armLimitOffset = mE.getCurrentPosition();
                        }
                    }

                    if ((potentiometer.getVoltage() - backIntakeUpright) >= toleranceU) {
                        mU.setPower(-scaleShift((potentiometer.getVoltage() - backIntakeUpright), 0.3, 4, 1, 0, 4) - 0.4);
                        telemetry.addData("backIntakenegative", (((((potentiometer.getVoltage() - backIntakeUpright) - 0) * (1 - 0)) / (4 - 0)) + 0));
                    } else if ((potentiometer.getVoltage() - backIntakeUpright) < toleranceU) {
                        mU.setPower(-1 + scaleShift((potentiometer.getVoltage() - backIntakeUpright), 0, 4, 0, 1, 8) + 0.4);
                        telemetry.addData("backIntakepositive", (-1 + scaleShift((potentiometer.getVoltage() - backIntakeUpright), 0, 4, 0, 1, 8) + 0.4));
                    } else {
                        mU.setPower(0);
                        intakeState = IntakeState.TEN_SEC_INTAKE;
                        armState = ArmState.FREE;
                    }

                    sVPosition = backIntakeWrist;
                    break;
                case BACK_DELIVERY:
                    toleranceU = 0.15;
                    toleranceE = 200;

                    if (armLimitOffset != 0) {
                        if (Math.abs(potentiometer.getVoltage() - deliveryUpright) < 0.5) { //prevent upright motor from being overstressed
                            if ((mE.getCurrentPosition() - armLimitOffset - deliveryExtension) >= toleranceE) {
                                mE.setPower(-scaleShift((mE.getCurrentPosition() - armLimitOffset - deliveryExtension), 0, 600, 1, 0, 1) - 0.2);//-1
                                telemetry.addData("-mE", scaleShift((mE.getCurrentPosition() - armLimitOffset - deliveryExtension), 0, 600, 1, 0, 1) + 0.2);
                            } else if ((mE.getCurrentPosition() - armLimitOffset - deliveryExtension) < toleranceE) {
                                mE.setPower(-scaleShift((mE.getCurrentPosition() - armLimitOffset - deliveryExtension), 0, 600, 1, 0, 1));//1
                                telemetry.addData("+mE", scaleShift((mE.getCurrentPosition() - armLimitOffset - deliveryExtension), 0, 600, 1, 0, 1) + 0.2);
                            } else {
                                mE.setPower(0);
                            }
                        }
                    } else {
                        mE.setPower(-0.5);
                        if (lAB.isPressed()) {
                            armLimitOffset = mE.getCurrentPosition();
                        }
                    }

                    if ((potentiometer.getVoltage() - deliveryUpright) >= toleranceU) {
                        mU.setPower(-scaleShift((potentiometer.getVoltage() - deliveryUpright), 0.3, 4, 1, 0, 4) - 0.4);
                        telemetry.addData("deliverynegative", (((((potentiometer.getVoltage() - deliveryUpright) - 0) * (1 - 0)) / (4 - 0)) + 0));

                    } else if ((potentiometer.getVoltage() - deliveryUpright) < toleranceU) {
                        mU.setPower(-1 + scaleShift((potentiometer.getVoltage() - deliveryUpright), 0, 4, 0, 1, 8) + 0.4);
                        telemetry.addData("deliverypositive", (-1 + scaleShift((potentiometer.getVoltage() - deliveryUpright), 0, 4, 0, 1, 8) + 0.4));
                    } else {
                        mU.setPower(0);
                        intakeState = IntakeState.OFF;
                        armState = ArmState.FREE;
                    }

                    sVPosition = deliveryWrist;
                    break;
                case CAP_INTAKE:
                    toleranceU = 0.1;
                    toleranceE = 100;

                    if (armLimitOffset != 0) {
                        if ((mE.getCurrentPosition() - armLimitOffset - capIntakeExtension) >= toleranceE) {
                            mE.setPower(-scaleShift((mE.getCurrentPosition() - armLimitOffset - capIntakeExtension), 0, 600, 1, 0, 1) - 0.2);//-1
                        } else if ((mE.getCurrentPosition() - armLimitOffset - capIntakeExtension) < toleranceE) {
                            mE.setPower(-scaleShift((mE.getCurrentPosition() - armLimitOffset - capIntakeExtension), 0, 600, 1, 0, 1));//1
                        } else {
                            mE.setPower(0);
                        }
                    } else {
                        mE.setPower(-0.5);
                        if (lAB.isPressed()) {
                            armLimitOffset = mE.getCurrentPosition();
                        }
                    }

                    if ((potentiometer.getVoltage() - capIntakeUpright) >= toleranceU) {
                        mU.setPower(-scaleShift((potentiometer.getVoltage() - capIntakeUpright), 0.3, 4, 1, 0, 4) - 0.4);
                        telemetry.addData("capIntakenegative", (((((potentiometer.getVoltage() - capIntakeUpright) - 0) * (1 - 0)) / (4 - 0)) + 0));

                    } else if ((potentiometer.getVoltage() - capIntakeUpright) < toleranceU) {
                        mU.setPower(-1 + scaleShift((potentiometer.getVoltage() - capIntakeUpright), 0, 4, 0, 1, 8) + 0.4);
                        telemetry.addData("capIntakepositive", (-1 + scaleShift((potentiometer.getVoltage() - capIntakeUpright), 0, 4, 0, 1, 8) + 0.4));
                    } else {
                        mU.setPower(0);
                        intakeState = IntakeState.TEN_SEC_INTAKE;
                        armState = ArmState.FREE;
                    }

                    sVPosition = capIntakeWrist;
                    break;
                case CAP_DELIVERY:
                    toleranceU = 0.1;
                    toleranceE = 100;

                    if (armLimitOffset != 0) {
                        if (Math.abs(potentiometer.getVoltage() - capDeliveryUpright) < 0.5) { //prevent upright motor from being overstressed
                            if ((mE.getCurrentPosition() - armLimitOffset - capDeliveryExtension) >= toleranceE) {
                                mE.setPower(-scaleShift((mE.getCurrentPosition() - armLimitOffset - capDeliveryExtension), 0, 600, 1, 0, 1) - 0.2);//-1
                                telemetry.addData("-mE", scaleShift((mE.getCurrentPosition() - armLimitOffset - capDeliveryExtension), 0, 600, 1, 0, 1) + 0.2);
                            } else if ((mE.getCurrentPosition() - armLimitOffset - capDeliveryExtension) < toleranceE) {
                                mE.setPower(-scaleShift((mE.getCurrentPosition() - armLimitOffset - capDeliveryExtension), 0, 600, 1, 0, 1));//1
                                telemetry.addData("+mE", scaleShift((mE.getCurrentPosition() - armLimitOffset - capDeliveryExtension), 0, 600, 1, 0, 1) + 0.2);
                            } else {
                                mE.setPower(0);
                            }
                        }
                    } else {
                        mE.setPower(-0.5);
                        if (lAB.isPressed()) {
                            armLimitOffset = mE.getCurrentPosition();
                        }
                    }

                    if ((potentiometer.getVoltage() - capDeliveryUpright) >= toleranceU) {
                        mU.setPower(-scaleShift((potentiometer.getVoltage() - capDeliveryUpright), 0.3, 4, 1, 0, 4) - 0.4);
                        telemetry.addData("capDeliverynegative", (((((potentiometer.getVoltage() - capDeliveryUpright) - 0) * (1 - 0)) / (4 - 0)) + 0));
                    } else if ((potentiometer.getVoltage() - capDeliveryUpright) < toleranceU) {
                        mU.setPower(-1 + scaleShift((potentiometer.getVoltage() - capDeliveryUpright), 0, 4, 0, 1, 8) + 0.4);
                        telemetry.addData("capDeliverypositive", (-1 + scaleShift((potentiometer.getVoltage() - capDeliveryUpright), 0, 4, 0, 1, 8) + 0.4));
                    } else {
                        mU.setPower(0);
                        intakeState = IntakeState.OFF;
                        armState = ArmState.FREE;
                    }

                    sVPosition = capDeliveryWrist;
                    break;
                case INIT:
                    toleranceU = 0.2;
                    toleranceE = 100;

                    if (armLimitOffset != 0) {
                        if (Math.abs(potentiometer.getVoltage() - initUpright) < 0.5) { //prevent upright motor from being overstressed
                            if ((mE.getCurrentPosition() - armLimitOffset - initExtension) >= toleranceE) {
                                mE.setPower(-scaleShift((mE.getCurrentPosition() - armLimitOffset - initExtension), 0, 600, 1, 0, 1) - 0.2);//-1
                                telemetry.addData("-mE", scaleShift((mE.getCurrentPosition() - armLimitOffset - initExtension), 0, 600, 1, 0, 1) + 0.2);
                            } else if ((mE.getCurrentPosition() - armLimitOffset - initExtension) < toleranceE) {
                                mE.setPower(-scaleShift((mE.getCurrentPosition() - armLimitOffset - initExtension), 0, 600, 1, 0, 1));//1
                                telemetry.addData("+mE", scaleShift((mE.getCurrentPosition() - armLimitOffset - initExtension), 0, 600, 1, 0, 1) + 0.2);
                            } else {
                                mE.setPower(0);
                            }
                        }
                    } else {
                        mE.setPower(-0.5);
                        if (lAB.isPressed()) {
                            armLimitOffset = mE.getCurrentPosition();
                        }
                    }

                    if ((potentiometer.getVoltage() - initUpright) >= toleranceU) {
                        mU.setPower(-scaleShift((potentiometer.getVoltage() - initUpright), 0.3, 4, 1, 0, 4) - 0.4);
                        telemetry.addData("initnegative", (((((potentiometer.getVoltage() - initUpright) - 0) * (1 - 0)) / (4 - 0)) + 0));

                    } else if ((potentiometer.getVoltage() - initUpright) < toleranceU) {
                        mU.setPower(-1 + scaleShift((potentiometer.getVoltage() - initUpright), 0, 4, 0, 1, 8) + 0.4);
                        telemetry.addData("initpositive", (-1 + scaleShift((potentiometer.getVoltage() - initUpright), 0, 4, 0, 1, 8) + 0.4));
                    } else {
                        mU.setPower(0);
                        intakeState = IntakeState.OFF;
                        armState = ArmState.FREE;
                    }

                    sVPosition = initWrist;
                    break;
                case FREE:
                    //sets arm extension and arm upright motion
                    mE.setPower(-leftY2); //also works for mF on Toby bot
                    mU.setPower(rightY2 * 0.8);//0.8 power multiplier
                    break;
            }

            // intake (in postitive, out negative)
            switch (intakeState) {
                case TEN_SEC_INTAKE:
                    sI.setPower(1);
                    if (intakeTimer.seconds() > 10) {
                        intakeState = IntakeState.OFF;
                    }
                    break;
                case INDEFININTE_INTAKE:
                    sI.setPower(1);
                    break;
                case FIVE_SEC_DELIVERY:
                    sI.setPower(-1);
                    if (intakeTimer.seconds() > 5) {
                        intakeState = IntakeState.OFF;
                    }
                    break;
                case INDEFININTE_DELIVERY:
                    sI.setPower(-1);
                    break;
                case TOGGLE:
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

            previousRightBumperState = rightBumper;
            previousBState = bButton;
            previousX2State = xButton2;
            extensionPosition = mE.getCurrentPosition();
        }


    }


}
