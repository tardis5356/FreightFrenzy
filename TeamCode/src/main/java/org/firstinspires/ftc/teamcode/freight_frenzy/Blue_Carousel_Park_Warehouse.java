package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Blue_Carousel_Park_Warehouse", group = "Autonomous")

public class Blue_Carousel_Park_Warehouse extends AutoBase_FF {

    ArrayList<String> steps = new ArrayList<>();
    //creates list of steps to be completed

    double targetZoneX = 0;
    double targetZoneY = 0;
    double lastTheta = 0;
    double myTime = 0;

    //initializes target zone variables, sets default target zone

    public void CreateSteps() {
//
        steps.add("DRIVE_TO_LIMIT");
        steps.add("FIND_ELEMENT_POSITION");
        steps.add("STRAFE_TO_CAROUSEL");//odometry
        steps.add("BACK_INTO_CAROUSEL");//time
//
        steps.add("SPIN_SPINNER");
//
        steps.add("DRIVE_TO_HUB_SU");//odometry
        steps.add("MOVE_ARM_UP");
        steps.add("MOVE_ARM");
        steps.add("DRIVE_TO_HUB_SU_2");//odometry
//
//
        steps.add("DROP_BLOCK");
//
        steps.add("BACK_AWAY_FROM_HUB");//odometry
//
//        //steps.add("RESET_ARM");//make arm straight up instead of horizontal
//
        steps.add("STRAFE_FROM_HUB");//odometry
        steps.add("RESET_ARM");//make arm straight up instead of horizontal

        steps.add("GO_NEAR_WAREHOUSE");//odometry
        steps.add("STRAFE_TSE");//odometry
        steps.add("RAISE_ODOMETERS");
        steps.add("GO_IN_WAREHOUSE");//time
        steps.add("PARK_IN_WAREHOUSE");//time
        steps.add("ROTATE_TO_180");//gyro
        steps.add("LOWER_ARM");

//        steps.add("PARK_IN_WAREHOUSE");//distance

        steps.add("STOP");


    }


    @Override
    public void runOpMode() {//Start of the initialization for autonomous

        boolean done = false;
        CreateSteps();

        //Init functions
        defineComponentsFred();

        double targetDistanceX = 0;
        double targetDistanceY = 0;
        double targetTheta = 0;
        String elementPosition = "NONE";

        double distanceTolerance = 2;
        double rotationTolerance = 1;
        double endDistanceTolerance = 1;
        double endRotationTolerance = 0.5;

        double rotationAngle = 0;
        boolean limitTriggered = false;

        boolean moveDone = false;
        double looseTolerance = 4;
        boolean firstCheck = false;
        double firstTime = 0;

        double targetX = 0;
        double targetY = 0;
        double rotationAngle1 = 0;
        double rotationAngle2 = 0;
        double tolerance = 1;
        double initialGyro = gyroZ;
        double finalGyro = 0;
        boolean check = false;
        boolean secondCheck = false;
        boolean thirdCheck = false;
        boolean fourthCheck = false;
        boolean fifthCheck = false;
        boolean sixthCheck = false;
        boolean eighthCheck = false;
        boolean seventhCheck = false;
        boolean ninthCheck = false;
        boolean angleDone = false;
        boolean extendDone = false;
        boolean angleUpDone = false;
        //initializing intake so that it's not powered
        sI.setPower(0);

        //instance fields/global variables

//            ////////////////setting robot in initialization, readies robot for autonomous  /////////////
        pipeline = new TeamElementPositionTest.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //Upright rotation works, do not set to sideways left
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        while (!opModeIsActive() && !isStopRequested()) {
            //adds vision recognition telemetry for debug and check
            elementPosition = TeamElementPositionTest.getPosition();
            telemetry.addData("Team Element Position", elementPosition);
            telemetry.addData("Hub level", hubLevel);
            telemetry.addData("Anti-Blueness", pipeline.getAnalysis());
            telemetry.update();
            //sets arm, extension, and wrist on initialization to get within 18 inches
            scrunchUpBot();
            lowerOdometerServos();
            if (elementPosition == "LEFT") {
                hubLevel = "BOTTOM";
            } else if (elementPosition == "CENTER") {
                hubLevel = "MIDDLE";
            } else if (elementPosition == "RIGHT") {
                hubLevel = "TOP";
            }
            changeHubLevel(elementPosition);
        }

        ////////////////above code runs in initialization, readies robot for autonomous  /////////////

        waitForStart();

        //Reset time variables
        runtime.reset();

        // loops through all steps the robot is supposed to perform
        for (String currentStep : steps) {
            done = false;      // reset so we can run the next step...

            // decides which step we are on, and runs that action
            while (opModeIsActive() && (done == false)) {

                telemetry.addData("current step", currentStep);
                fredTelemetry();
                telemetry.update();
                //Update global sensor values
                updatePoseStrafe();
                gyroUpdate();

//               readDistanceSensors();
                leftDistance = Range.clip(rangeSensorLeft.getDistance(DistanceUnit.INCH), 0, 200);
                frontDistance = Range.clip(rangeSensorFront.getDistance(DistanceUnit.INCH), 0, 200);

                //sets distance sensors to a small negative number if sensors read not a number -- this is necessary when the robot is too close to a wall
                if (Double.isNaN(leftDistance)) {

                    leftDistance = -2;
                }
                if (Double.isNaN(frontDistance)) {

                    frontDistance = -2;
                }

                leftDistanceArray = popValueIntoArray(leftDistanceArray, leftDistance);
                frontDistanceArray = popValueIntoArray(frontDistanceArray, frontDistance);

                leftDistanceFiltered = median(leftDistanceArray);
                frontDistanceFiltered = median(frontDistanceArray);

                switch (currentStep) {

                    case ("PARK_IN_WAREHOUSE"):
                        if (runtime.seconds() > 0 && runtime.seconds() < 0.5) {
                            drive(-0.5, 0.7, 0);
                            done = false;
                        } else {
                            done = true;
                            changeStep();
                        }
                        break;

                    case ("STRAFE_TO_CAROUSEL"):
                        targetX = 26;
                        targetY = 11;
                        targetTheta = 0;
                        done = (moveToLocationOdometry(targetX, targetY, targetTheta, 3, rotationTolerance));
                        break;

                    case ("BACK_INTO_CAROUSEL"):
                        if (runtime.seconds() > 0 && runtime.seconds() < 1.5) {
                            drive(0.5, -0.4, 0);
                            done = false;
                        } else {
                            done = true;
                            changeStep();
                        }
                        break;
                    case ("DRIVE_TO_HUB_SU"):
                        targetX = 22;
                        targetY = 42;
                        targetTheta = 0;
                        done = (moveToLocationOdometry(targetX, targetY, targetTheta, 3, rotationTolerance));
                        break;

                    case ("DRIVE_TO_HUB_SU_2"):
                        targetX = -4;
                        targetY = 42;
                        targetTheta = 90;
                        done = (moveToLocationOdometry(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;

                    case ("BACK_AWAY_FROM_HUB"):
                        targetX = 24;
                        targetY = 40;
                        targetTheta = 90;
                        done = (moveToLocationOdometry(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;

                    case ("STRAFE_FROM_HUB"):
                        targetX = 24;
                        targetY = 12;
                        targetTheta = 90;
                        done = (moveToLocationOdometry(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;
                    case ("GO_NEAR_WAREHOUSE"):
                        targetX = -44;
                        targetY = 12;
                        targetTheta = 90;
                        done = (moveToLocationOdometry(targetX, targetY, targetTheta, 3, rotationTolerance));
                        break;
                    case ("STRAFE_TSE"):
                        targetX = -44;
                        targetY = 16;
                        targetTheta = 90;
                        done = (moveToLocationOdometry(targetX, targetY, targetTheta, 3, rotationTolerance));
                        break;
                    case ("RAISE_ODOMETERS"):
                        if (runtime.seconds() > 0 && runtime.seconds() < 1) {
                            raiseOdometerServos();
                        } else {
                            done = true;
                            changeStep();
                        }
                        break;

                    case ("GO_IN_WAREHOUSE"):
                        if (runtime.seconds() > 0 && runtime.seconds() < 1.25) {
                            drive(-1, 0, 0);
                            done = false;
                        } else {
                            done = true;
                            changeStep();
                        }
                        break;
//                    case ("PARK_IN_WAREHOUSE"):
//                        targetDistanceX = 0;
//                        targetDistanceY = 5;
//                        done = (moveToLocationDistance(targetDistanceX, targetDistanceY, 1, "", "frontDistance", 90, 5));
//                        break;


                    case ("RESET_ARM"):
                        angleDone = false;
                        extendDone = false;
                        double potTolerance = 0.1;
                        drive(0, 0, 0);
                        double armAngleBack = armHorizontal - 1.15;
//        telemetry.addData("target arm angle", armAngle);
//        telemetry.addData("target arm extension", armReach);
//        telemetry.addData("arm extension", mE.getCurrentPosition());
//        telemetry.addData("telescope pose (offset)", telescopePose);
                        sV.setPosition(0.1);
                        if (lAB.isPressed()) {  //uses limit switch to move arm to a known position
                            telescopePose = mE.getCurrentPosition();
                            mE.setPower(0);
                            extendDone = true;
                        } else if (!lAB.isPressed()) {
                            mE.setPower(-1);
                        }

                        if ((Math.abs(potentiometer.getVoltage() - armAngleBack) > potTolerance) && extendDone) {
                            if (potentiometer.getVoltage() > armAngleBack) {

                                mU.setPower(-0.5);
                            } else if (potentiometer.getVoltage() < armAngleBack) {

                                mU.setPower(0.5);
                            }
                        } else {

                            mU.setPower(0);
                            angleDone = true;

                        }
                        if (angleDone && extendDone) {

                            done = true;
                            changeStep();

                        }
                        changeStep();

                        break;

                    case ("LOWER_ARM"):
                        angleDone = false;
                        extendDone = false;
                        potTolerance = 0.1;
                        drive(0, 0, 0);
                        armAngleBack = armHorizontal + 0.85;
//        telemetry.addData("target arm angle", armAngle);
//        telemetry.addData("target arm extension", armReach);
//        telemetry.addData("arm extension", mE.getCurrentPosition());
//        telemetry.addData("telescope pose (offset)", telescopePose);
                        sV.setPosition(0.51 + wristStraight);
                        if (lAB.isPressed()) {  //uses limit switch to move arm to a known position
                            telescopePose = mE.getCurrentPosition();
                            mE.setPower(0);
                            extendDone = true;
                        } else if (!lAB.isPressed()) {
                            mE.setPower(-1);
                        }

                        if ((Math.abs(potentiometer.getVoltage() - armAngleBack) > potTolerance) && extendDone) {
                            if (potentiometer.getVoltage() > armAngleBack) {

                                mU.setPower(-0.5);
                            } else if (potentiometer.getVoltage() < armAngleBack) {

                                mU.setPower(0.5);
                            }
                        } else {

                            mU.setPower(0);
                            angleDone = true;

                        }
                        if (angleDone && extendDone) {

                            done = true;
                            changeStep();

                        }
                        changeStep();

                        break;

                    case "DROP_BLOCK":
                        if (!sixthCheck) {
                            //if this step has not been run before, sets myTime to the runtime
                            myTime = runtime.seconds();
                            sixthCheck = true;
                        }
                        double sixthTime = 1;

                        if ((runtime.seconds() - myTime) <= sixthTime) {
                            sI.setPower(0.5);
                        }
                        if ((runtime.seconds() - myTime) > sixthTime) {
                            sI.setPower(0);
                            done = true;
                            changeStep();
                        }
                        break;
                    case "MOVE_ARM_UP":
                        potTolerance = 0.1;
                        if (Math.abs(potentiometer.getVoltage() - armUp) > potTolerance) {
                            if (potentiometer.getVoltage() > armUp) {

                                mU.setPower(-0.6);
                            } else if (potentiometer.getVoltage() < armUp) {

                                mU.setPower(0.6);
                            }
                        } else {
                            mU.setPower(0);
                            done = true;
                            changeStep();

                        }

                        break;

                    case "MOVE_ARM":
                        //127 extension ticks per cm
                        drive(0, 0, 0);
                        potTolerance = 0.05;
                        angleDone = false;
                        extendDone = false;
                        //armReach = 3200;
                        telemetry.addData("hub level", hubLevel);
                        telemetry.addData("target arm angle", armAngle);
                        telemetry.addData("target arm extension", armReach);
                        telemetry.addData("arm extension", mE.getCurrentPosition());
                        telemetry.addData("telescope pose (offset)", telescopePose);
                        sV.setPosition(wristPosition);
                        //points arm upward so arm can extend

                        double reachTolerance = 50;
                        //Extension moves only if arm is pointed up
                        if (Math.abs(mE.getCurrentPosition() - armReach) > reachTolerance) {
                            if (mE.getCurrentPosition() > armReach) {

                                mE.setPower(-1);
                            } else if (mE.getCurrentPosition() < armReach) {

                                mE.setPower(1);
                            }
                        } else {

                            mE.setPower(0);
                            extendDone = true;
                        }

                        telemetry.addData("extend done", extendDone);
                        //Angle of arm only moves when extension is done
                        if ((Math.abs(potentiometer.getVoltage() - armAngle) > potTolerance) && extendDone) {
                            if (potentiometer.getVoltage() > armAngle) {

                                mU.setPower(-0.5);
                            } else if (potentiometer.getVoltage() < armAngle) {

                                mU.setPower(0.5);
                            }
                        } else {

                            mU.setPower(0);
                            angleDone = true;

                        }

                        telemetry.addData("arm done", angleDone);


                        if (angleDone && extendDone) {

                            done = true;
                            changeStep();
                        }

                        break;

                    case "DRIVE_TO_LIMIT":
                        if (lAB.isPressed()) {  //uses limit switch to move arm to a known position
                            limitTriggered = true;
                            mE.setPower(0);
                            telescopePose = mE.getCurrentPosition();
                            telemetry.addData("telescopePose", telescopePose);
                            done = true;
                            changeStep();
                        } else if (!limitTriggered) {
                            mE.setPower(-0.65);
                        }
                        break;

                    case "SPIN_SPINNER":
                        if (runtime.seconds() > 0 && runtime.seconds() < 2) {
                            mSR.setPower(0.5);
                            done = false;
                        } else {
                            mSR.setPower(0);
                            done = true;
                            changeStep();
                        }
                        break;


                    case "FIND_ELEMENT_POSITION":
                        //changes the target zone that the robot moves to based on the number of rings counted during initialization
                        changeHubLevel(hubLevel);
                        done = true;
                        changeStep();
                        break;

                    case "Test":
                        //used for testing odometry
                        targetX = 0;
                        targetY = 12;
                        targetTheta = 0;
                        //done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;

                    case "WAIT":
                        //wait step that can be added in if necessary for a debug of the code
                        if (runtime.seconds() > 0 && runtime.seconds() < 5) {
                            done = false;
                        } else {
                            done = true;
                            changeStep();
                        }
                        break;

                    case "STOP":
                        //stops drive train
                        drive(0, 0, 0);
                        done = true;
                        changeStep();
                        break;

                    case "ROTATE_TO_180":
                        rotationAngle1 = 180;
                        tolerance = 5;
                        //totalAngleChange (second variable) cannot be 0
                        //preciseRotationChange(rotationAngle1, rotationAngle1);
                        rotateSigmoid(rotationAngle1);
                        if (Math.abs(gyroZ - rotationAngle1) < tolerance) {
                            done = true;
                            //finalGyro = gyroZ;
                            changeStep();
                        }
                        break;

                    case "ROTATE_TO_0":
                        rotationAngle1 = 0;
                        tolerance = 5;
                        //totalAngleChange (second variable) cannot be 0
                        //preciseRotationChange(rotationAngle1, rotationAngle1);
                        rotateSigmoid(rotationAngle1);
                        if (Math.abs(gyroZ - rotationAngle1) < tolerance) {
                            done = true;
                            //finalGyro = gyroZ;
                            changeStep();
                        }
                        break;

                    case ("DRIVE_FORWARD"):

                        if (!check) {
                            //if this step has not been run before, sets myTime to the runtime
                            myTime = runtime.seconds();
                            check = true;
                        }
                        double time = 0.75;

                        if ((runtime.seconds() - myTime) <= time) {
                            drive(-1, 0, 0);
                        }
                        if ((runtime.seconds() - myTime) > time) {
                            done = true;
                            changeStep();
                        }

                        break;

                }
            }
        }
        shutdown();
    }

    public boolean moveToLocationDistance
        //function for setting a point with odometry, changes step when robot position is in a certain tolerance
    (double targetDistanceX, double targetDistanceY, double distanceTolerance, String sensorForX, String sensorForY, double targetTheta, double tolTheta) {
        if (isInToleranceDistance(targetDistanceX, targetDistanceY, targetTheta, distanceTolerance, tolTheta)) {
            lastTheta = gyroZ;
            changeStep(); // you are within tolerance, so stop the drive train and move to next step
            return true;
        } else {
            telemetry.addData("target x", targetDistanceX);
            telemetry.addData("target y", targetDistanceY);
            moveToDistFromWall(targetDistanceX, targetDistanceY, sensorForX, sensorForY, targetTheta, tolTheta);
            return false;
        }

    }

    public boolean moveToLocationOdometry
        //function for setting a point with odometry, changes step when robot position is in a certain tolerance
    (double targetX, double targetY, double targetTheta, double distanceTolerance, double rotationTolerance) {
        if (isInTolerance(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance)) {
            lastTheta = gyroZ;
            changeStep(); // you are within tolerance, so stop the drive train and move to next step
            return true;
        } else {
            telemetry.addData("target x", targetX);
            telemetry.addData("target y", targetY);
            moveToPose(targetX, targetY, targetTheta, 50);
            return false;
        }

    }
}






