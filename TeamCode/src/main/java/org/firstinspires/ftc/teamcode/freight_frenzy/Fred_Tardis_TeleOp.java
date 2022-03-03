package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
        BOTTOM_CAP_DELIVERY,
        TOP_CAP_DELIVERY,
        BACK_MID_DELIVERY,
        NEUTRAL,
        FREE
    }


    ArmState armState = ArmState.NEUTRAL;

    public enum IntakeState {
        TEN_SEC_INTAKE,
        INDEFININTE_INTAKE,
        FIVE_SEC_DELIVERY,
        INDEFININTE_DELIVERY,
        FREE,
        OFF
    }

    IntakeState intakeState = IntakeState.OFF;

    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime spinnerTimer = new ElapsedTime();
    ElapsedTime distanceTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        defineComponentsFred();
        double powerMultiplier = 0.6;
        boolean previousBState = false;
        boolean motorPowerFast = false;

        boolean extensionReset = false;
        double sVPosition = sV.getPosition();
        boolean intakeStateSet = false;

        String teleopMode = "allianceHub"; //allianceHub, sharedHub, endgame

        //runtime
        int runtimeRounded = 0;
        boolean teleopStarted = false;

        boolean armFreed = false;

        //spinner
        int spinnerLevel = 0; //current level of power
        double[] spinnerPowers = {0.5, 0.6, 1}; //levels of each power shift
        double[] spinnerLevelTimes = {0.25, 1}; //times spinner power shifts
        double spinnerPower = spinnerPowers[spinnerLevel]; //current power

        //arm automation presets
        //upright
        double neutralUpright = 1.1;
        double initUpright = neutralUpright + 1.8;
        double capIntakeUpright = neutralUpright + 2.13;
        double bottomCapDeliveryUpright = neutralUpright + 0.53;
        double topCapDeliveryUpright = neutralUpright + 0.41;
        double backDeliveryUpright = neutralUpright - 0.3;
        double backIntakeUpright = neutralUpright - 0.85;
        double midBackDeliveryUpright = neutralUpright - 0.76;

        //wrist
        double neutralWrist = 0.67;
        double initWrist = neutralWrist - 0.6;
        double capIntakeWrist = neutralWrist - 0.12;
        double bottomCapDeliveryWrist = neutralWrist - 0.41;
        double topCapDeliveryWrist = neutralWrist - 0.38;
        double backDeliveryWrist = neutralWrist + 0.2;
        double backIntakeWrist = neutralWrist + 0.2;
        double midBackDeliveryWrist = neutralWrist + 0.33;

        //extension
        double neutralExtension = 100;
        double initExtension = 100;
        double capIntakeExtension = 500;
        double capDeliveryExtension = 1000;
        double backDeliveryExtension = 800;
        double backIntakeExtension = 600;
        double midBackDeliveryExtension = 400;
        int extensionTolerance = 50;

        intakeTimer.reset();
        spinnerTimer.reset();
        distanceTimer.reset();
        runtime.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updatePoseStrafe();
            gyroUpdate();

            if (!teleopStarted) {
                runtime.reset();
                teleopStarted = true;
            }

            //Gamepad 1 Variables
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

            //general
            telemetry.addData("gyro", "" + String.format("%.2f deg", gyroZ));
            telemetry.addData("Motor power fast", motorPowerFast);

            //wrist
            telemetry.addData("Wrist angle", sVPosition);

            //arm
            telemetry.addData("potentiometer voltage", potentiometer.getVoltage());
            telemetry.addData("arm state", armState);

            //extension
            telemetry.addData("extensionReset", extensionReset);
            telemetry.addData("mE mode", mE.getMode());
            telemetry.addData("mE current position", mE.getCurrentPosition());
            telemetry.addData("arm limit", lAB.isPressed());

            //intake
            telemetry.addData("intakeState", intakeState);
            telemetry.addData("intakeTimer", intakeTimer);
            telemetry.addData("intakeDistance", dI.getDistance(DistanceUnit.CM));

            //spinner
            telemetry.addData("spinnerTimer", spinnerTimer.seconds());
            telemetry.addData("spinnerLevel", spinnerLevel);
            telemetry.addData("spinnerPower", spinnerPower);

            //runtime
            telemetry.addData("runtime", runtime.seconds());
            telemetry.addData("runtime", Math.round(runtime.seconds()));
            telemetry.addData("runtimeRounded", runtimeRounded);

            telemetry.update();

            //drives robot
            drive(leftY1, leftX1, rightX1);

            //changes drive speed
            if (bButton != previousBState && bButton) {
                if (motorPowerFast) {
                    powerMultiplier = 0.5;
                    motorPowerFast = false;
                } else {
                    powerMultiplier = 1;
                    motorPowerFast = true;
                }
            }

            //deploy odometers
            if (xButton) {
                lowerOdometerServos();
            } else {
                raiseOdometerServos();
            }

            //rumble
            runtimeRounded = Math.toIntExact(Math.round(runtime.seconds()));

            if (gamepad1.dpad_up) {
                teleopMode = "endgame";
            } else if (gamepad1.dpad_right) {
                teleopMode = "allianceHub";
            } else if (gamepad1.dpad_left) {
                teleopMode = "sharedHub";
            }

            if (teleopMode == "endgame") {
                if (runtimeRounded % 2 == 0 || runtimeRounded % 2 == 0.5) {
                    led1green.setState(false);
                    led1red.setState(false);
                } else {
                    led1green.setState(true);
                    led1red.setState(true);
                }
            } else if (teleopMode == "allianceHub") {
                if (dI.getDistance(DistanceUnit.CM) < 7) {
                    led1green.setState(false);
                    led1red.setState(true);
                } else {
                    led1green.setState(true);
                    led1red.setState(false);
                }
            } else if (teleopMode == "sharedHub") {
                if (dI.getDistance(DistanceUnit.CM) < 7) {
                    if (runtimeRounded % 2 == 0 || runtimeRounded % 2 == 0.5) {
                        led1green.setState(false);
                        led1red.setState(false);
                    } else {
                        led1green.setState(false);
                        led1red.setState(true);
                    }
                } else {
                    if (runtimeRounded % 2 == 0 || runtimeRounded % 2 == 0.5) {
                        led1green.setState(false);
                        led1red.setState(false);
                    } else {
                        led1green.setState(true);
                        led1red.setState(false);
                    }
                }
            }

            if (dI.getDistance(DistanceUnit.CM) < 7) {
                if (!gamepad2.isRumbling()) {
                    gamepad1.rumbleBlips(2);
                    gamepad2.rumbleBlips(2);
                }
                if (rightY2 == 0 && distanceTimer.seconds() > 1.2) {
                    if (teleopMode == "allianceHub") {
                        armState = ArmState.BACK_DELIVERY; //auto delivery position alliance
                        intakeState = IntakeState.FREE;
                    } else if (teleopMode == "sharedHub") {
                        armState = ArmState.BACK_MID_DELIVERY; //auto delivery position shared
                        intakeState = IntakeState.FREE;
                    }
                }
            } else {
                distanceTimer.reset();
            }


            switch (runtimeRounded) {
                case 30://30s
                case 60://60s
                case 100://10s into endgame
                case 106://26s into endgame (duck benchmark)
                case 110://20s into endgame
                    gamepad1.rumble(20);
                    gamepad2.rumble(20);
                    break;
                case 80://10s until endgame
                    gamepad1.rumbleBlips(1);
                    gamepad2.rumbleBlips(1);
                    break;
                case 90://endgame
                    gamepad1.rumbleBlips(3);
                    gamepad2.rumbleBlips(3);
                    break;
                case 115://5s until match over
                    gamepad1.rumble(500);
                    gamepad2.rumble(500);
                    break;
                case 120://end
                    gamepad1.rumble(2000);
                    gamepad2.rumble(2000);
                    break;
            }

            //controls wrist up-down motion
            if ((rightBumper2)) {//&& (sWVPosition < 1))
                sVPosition += .05;
            } else if ((leftBumper2)) { //&& (sWVPosition > 0))
                sVPosition -= .05;
            }
            sV.setPosition(Range.clip(sVPosition, 0.01, 1));
            sVPosition = sV.getPosition();

            //resets extension encoder values
            if (!extensionReset && runtime.seconds() > 1) {
                mE.setPower(-1);
                if (lAB.isPressed()) {
                    mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extensionReset = true;
                }
            } else if (extensionReset) {
                mE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (mE.getCurrentPosition() < -50) {
                extensionReset = false;
            }

            //arm
            if (!gamepad2.start) {
                if (yButton2) {// neutral position
                    armState = ArmState.NEUTRAL;
                } else if (bButton2) {// backDelivery position
                    if (teleopMode == "allianceHub") {
                        armState = ArmState.BACK_DELIVERY;
                    } else if (teleopMode == "sharedHub") {
                        armState = ArmState.BACK_MID_DELIVERY;
                    }
                } else if (xButton2) {// backIntake position
                    armState = ArmState.BACK_INTAKE;
                } else if (aButton2) {// capIntake position
                    armState = ArmState.CAP_INTAKE;
                } else if (dpadLeft2) {// top cap
                    armState = ArmState.TOP_CAP_DELIVERY;
                    teleopMode = "endgame";
                } else if (dpadRight2) {// bottom cap
                    armState = ArmState.BOTTOM_CAP_DELIVERY;
                    teleopMode = "endgame";
                }

                if (gamepad2.y || gamepad2.b || gamepad2.a || gamepad2.x || gamepad2.dpad_right || gamepad2.dpad_left) {
                    armFreed = false;
                    intakeStateSet = false;
                }
                if (gamepad2.right_stick_y != 0 || gamepad2.left_stick_y != 0) {
                    armFreed = true;
                }

                if (extensionReset && !armFreed) {
                    switch (armState) {
                        case NEUTRAL:
                            armToPosPID(neutralUpright);
                            extensionToPos(neutralExtension, extensionTolerance);
                            sVPosition = neutralWrist;
                            if (!intakeStateSet) {
                                intakeState = IntakeState.FREE;
                                intakeStateSet = true;
                            }
                            break;
                        case BACK_INTAKE:
                            armToPosPID(backIntakeUpright);
                            extensionToPos(backIntakeExtension, extensionTolerance);
                            sVPosition = backIntakeWrist;
                            if (!intakeStateSet) {
                                intakeState = IntakeState.INDEFININTE_INTAKE;
                                intakeStateSet = true;
                            }
                            break;
                        case BACK_DELIVERY:
                            armToPosPID(backDeliveryUpright);
                            extensionToPos(backDeliveryExtension, extensionTolerance);
                            sVPosition = backDeliveryWrist;
                            if (!intakeStateSet) {
                                intakeState = IntakeState.TEN_SEC_INTAKE;
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
                        case BOTTOM_CAP_DELIVERY:
                            armToPosPID(bottomCapDeliveryUpright);
                            extensionToPos(capDeliveryExtension, extensionTolerance);
                            sVPosition = bottomCapDeliveryWrist;
                            if (!intakeStateSet) {
                                intakeState = IntakeState.TEN_SEC_INTAKE;
                                intakeStateSet = true;
                            }
                            break;
                        case TOP_CAP_DELIVERY:
                            armToPosPID(topCapDeliveryUpright);
                            extensionToPos(capDeliveryExtension, extensionTolerance);
                            sVPosition = topCapDeliveryWrist;
                            if (!intakeStateSet) {
                                intakeState = IntakeState.TEN_SEC_INTAKE;
                                intakeStateSet = true;
                            }
                            break;
                        case BACK_MID_DELIVERY:
                            armToPosPID(midBackDeliveryUpright);
                            extensionToPos(midBackDeliveryExtension, extensionTolerance);
                            sVPosition = midBackDeliveryWrist;
                            if (!intakeStateSet) {
                                intakeState = IntakeState.TEN_SEC_INTAKE;
                                intakeStateSet = true;
                            }
                            break;
                        case INIT:
                            armToPosPID(initUpright);
                            extensionToPos(initExtension, extensionTolerance);
                            sVPosition = initWrist;
                            if (!intakeStateSet) {
                                intakeState = IntakeState.FREE;
                                intakeStateSet = true;
                            }
                            break;
                    }
                }

                switch (armState) {
                    case FREE:
                        //sets arm extension and arm upright motion
                        mE.setPower(-leftY2); //also works for mF on Toby bot
                        mU.setPower(rightY2 * 0.5);//0.8 power multiplier
                        break;
                }

                // intake (in postitive, out negative)
                switch (intakeState) {
                    case TEN_SEC_INTAKE:
                        sI.setPower(-1);
                        if (intakeTimer.seconds() > 10) {
                            intakeState = IntakeState.FREE;
                        }
                        break;
                    case INDEFININTE_INTAKE:
                        sI.setPower(-1);
                        break;
                    case FIVE_SEC_DELIVERY:
                        sI.setPower(1);
                        if (intakeTimer.seconds() > 5) {
                            intakeState = IntakeState.FREE;
                        }
                        break;
                    case INDEFININTE_DELIVERY:
                        sI.setPower(1);
                        break;
                    case FREE:
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

                //auto arm overrides
                if ((rightY2 != 0) || (leftY2 != 0) || (rightBumper2) || (leftBumper2)) {
                    armState = ArmState.FREE;
                    armFreed = true;
                }
                //auto intake overrides
                if ((rightTrigger2 != 0) || (leftTrigger2 != 0)) {
                    intakeState = IntakeState.FREE;
                }

            }

            //carousel
            if (rightTrigger > 0.5 || leftTrigger > 0.5) {
                if (spinnerLevel < spinnerLevelTimes.length) {
                    if (spinnerTimer.seconds() > spinnerLevelTimes[spinnerLevel]) {
                        spinnerLevel++;
                        spinnerPower = spinnerPowers[spinnerLevel];
                        gamepad1.rumble(100);
                    }
                } else {
                    spinnerPower = 1;
                }
                if (rightTrigger > 0.5) {
                    mSL.setPower(spinnerPower);
                    mSR.setPower(1);
                } else if (leftTrigger > 0.5) {
                    mSL.setPower(-spinnerPower);
                    mSR.setPower(-1);
                }
            } else {
                spinnerTimer.reset();
                spinnerLevel = 0;
                spinnerPower = spinnerPowers[spinnerLevel];
                telemetry.addData("spinner off", spinnerTimer.seconds());
                mSL.setPower(0);
                mSR.setPower(0);
            }

            previousBState = bButton;
        }


    }


}
