package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Blue_Carousel_Autonomous", group = "Autonomous")

public class Blue_Carousel_Autonomous extends AutoBase_FF {

    ArrayList<String> steps = new ArrayList<>();
    //creates list of steps to be completed

    double targetZoneX = 0;
    double targetZoneY = 0;
    double lastTheta = 0;
    double myTime = 0;

    //initializes target zone variables, sets default target zone

    public void CreateSteps() {

                steps.add("DRIVE_TO_LIMIT");
                steps.add("FIND_ELEMENT_POSITION");
                steps.add("DRIVE_FROM_WALL");
                steps.add("MOVE_FROM_WALL");
               steps.add("ROTATE_TO_90");
                steps.add("MOVE_TO_CAROUSEL");
                steps.add("MOVE_TO_CAROUSEL_2");
//                steps.add("MOVE_TO_CAROUSEL_3");
                steps.add("SPIN_SPINNER");
                steps.add("DRIVE_TO_HUB_TIME");
                //steps.add("WAIT");
                steps.add("DRIVE_TO_HUB");
                 steps.add("MOVE_ARM");
                steps.add("ROTATE_TO_0");
                steps.add("DRIVE_FORWARD_TO_HUB");
                steps.add("DROP_BLOCK");
                steps.add("MOVE_BACKWARD");
                steps.add("RESET_ARM");
                steps.add("ROTATE_TO_90");
                steps.add("DRIVE_BACK");
                steps.add("PARK_IN_STORAGE_UNIT");
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
        //initializing intake so that it's not powered
        sI.setPower(0);
        //double telescopePose = 0;

        //instance fields/global variables

//            ////////////////setting robot in initialization, readies robot for autonomous  /////////////
        pipeline = new TeamElementPositionTest.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //Upright rotation works, do not set to sideways left
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
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

            if (elementPosition == "LEFT") {
                hubLevel = "BOTTOM";
            } else if (elementPosition == "CENTER") {
                hubLevel = "MIDDLE";
            } else if (elementPosition == "RIGHT"){
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
//                    telemetry.addData("last theta", lastTheta);
//                    telemetry.addData("potentiometer angle", getElevAngle(potentiometer.getVoltage()));
                telemetry.addData("gyro", "" + String.format("%.2f deg", gyroZ));
//                    telemetry.addData("current position x", "" + String.format("%.2f in.", pose.x));
//                    telemetry.addData("current position y", "" + String.format("%.2f in.", pose.y));
//                    telemetry.addData("arm position", mA.getCurrentPosition());
//                    telemetry.addData("wrist position", sW.getPosition());
//                    telemetry.addData("gripper position", sG.getPosition());
//                    telemetry.addData("runtime two", runtimeTwo);
                telemetry.addData("left distance (in)", "" + String.format("%.2f", leftDistance));
                telemetry.addData("right distance (in)", "" + String.format("%.2f", rightDistance));
                //telemetry.addData("back left distance", "" + String.format("%.2f cm", backLeftDistance));
                telemetry.addData("back right distance (in)", "" + String.format("%.2f", backDistance));
                telemetry.addData("front distance (in)", "" + String.format("%.2f", frontDistance));
                telemetry.addData("potentiometer angle", potentiometer.getVoltage());
                telemetry.addData("Hub level", hubLevel);
                telemetry.update();
                //Update global sensor values
                updatePoseStrafe();
                gyroUpdate();

                leftDistance = Range.clip(rangeSensorLeft.getDistance(DistanceUnit.INCH), 0, 200);
                rightDistance = Range.clip(rangeSensorRight.getDistance(DistanceUnit.INCH), 0, 200);
                backDistance = Range.clip(rangeSensorBack.getDistance(DistanceUnit.INCH), 0, 200);
                frontDistance = Range.clip(rangeSensorFront.getDistance(DistanceUnit.INCH), 0, 200);

                //sets distance sensors to a small negative number if sensors read not a number -- this is necessary when the robot is too close to a wall
                if (Double.isNaN(leftDistance)) {

                    leftDistance = -2;
                }
                if (Double.isNaN(rightDistance)) {

                    rightDistance = -2;
                }
                if (Double.isNaN(backDistance)) {

                    backDistance = -2;
                }
                if (Double.isNaN(frontDistance)) {

                    frontDistance = -2;
                }

                switch (currentStep) {

                    case("DRIVE_BACK"):
                        if (!ninthCheck) {
                            //if this step has not been run before, sets myTime to the runtime
                            myTime = runtime.seconds();
                            ninthCheck = true;
                        }
                        double ninthTime = 1.9;

                        if ((runtime.seconds() - myTime) <= ninthTime ){
                            drive(0.4,-0.2,0);
                        }
                        if ((runtime.seconds() - myTime) > ninthTime) {
                            drive(0,0,0);
                            done = true;
                            changeStep();
                        }
                        break;

                    case("MOVE_BACKWARD"):
                        targetDistanceX = 0;
                        targetDistanceY = 10;
                        done = (moveToLocation(targetDistanceX, targetDistanceY, 2, "", "backDistance", 0, 5));
                        break;

                    case("RESET_ARM"):
                        drive(0,0,0);
                        double potTolerance = 0.05;
                        boolean angleDone = false;
                        boolean extendDone = false;
                        double armAngleBack = armHorizontal;
                        telemetry.addData("target arm angle", armAngle);
                        telemetry.addData("target arm extension", armReach);
                        telemetry.addData("arm extension", mE.getCurrentPosition());
                        telemetry.addData("telescope pose (offset)", telescopePose);
                        sV.setPosition(0);
                        if(lAB.isPressed()) {  //uses limit switch to move arm to a known position
                            telescopePose = mE.getCurrentPosition();
                            mE.setPower(0);
                            extendDone = true;
                        }

                        else if (!lAB.isPressed()) {
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
                        telemetry.addData("arm done", angleDone);
                        if(angleDone && extendDone){

                            done = true;
                            changeStep();
                        }
                        break;


                    case("MOVE_TO_CAROUSEL_3"):
                        if (!seventhCheck) {
                            //if this step has not been run before, sets myTime to the runtime
                            myTime = runtime.seconds();
                            seventhCheck = true;
                        }
                        double seventhTime = 1;

                        if ((runtime.seconds() - myTime) <= seventhTime ){
                            drive(0.2,0.5,0);
                        }
                        if ((runtime.seconds() - myTime) > seventhTime) {
                            drive(0,0,0);
                            done = true;
                            changeStep();
                        }
                        break;

                    case("DRIVE_TO_HUB_TIME"):
                        if (!eighthCheck) {
                            //if this step has not been run before, sets myTime to the runtime
                            myTime = runtime.seconds();
                            eighthCheck = true;
                        }
                        double eighthTime = 1.3;

                        if ((runtime.seconds() - myTime) <= eighthTime ){
                            drive(-0.5,0,0);
                        }
                        if ((runtime.seconds() - myTime) > eighthTime) {
                            drive(0,0,0);
                            done = true;
                            changeStep();
                        }
                        break;

                    case("DRIVE_FORWARD_TO_HUB"):
                        targetDistanceX = 0;
                        targetDistanceY = 18;
                        done = (moveToLocation(targetDistanceX, targetDistanceY, 2, "", "backDistance", 0, 5));
                        break;


                    case "DROP_BLOCK":
                        if (!sixthCheck) {
                            //if this step has not been run before, sets myTime to the runtime
                            myTime = runtime.seconds();
                            sixthCheck = true;
                        }
                        double sixthTime = 2;

                        if ((runtime.seconds() - myTime) <= sixthTime ){
                            sI.setPower(0.5);
                        }
                        if ((runtime.seconds() - myTime) > sixthTime) {
                            sI.setPower(0);
                            done = true;
                            changeStep();
                        }
                        break;

                    case "PARK_IN_STORAGE_UNIT":
                        targetDistanceX = 22;
                        targetDistanceY = 1;
                        done = (moveToLocation(targetDistanceX, targetDistanceY, 1, "leftDistance", "backDistance", 90, 5));
                        break;

                    case "MOVE_ARM":
                        //43 extension ticks per cm
                        drive(0,0,0);
                        potTolerance = 0.05;
                        angleDone = false;
                        extendDone = false;
                        telemetry.addData("hub level", hubLevel);
                        telemetry.addData("target arm angle", armAngle);
                        telemetry.addData("target arm extension", armReach);
                        telemetry.addData("arm extension", mE.getCurrentPosition());
                        telemetry.addData("telescope pose (offset)", telescopePose);
                        sV.setPosition(wristPosition);
                        if (Math.abs(potentiometer.getVoltage() - armAngle) > potTolerance) {
                            if (potentiometer.getVoltage() > armAngle) {

                                mU.setPower(-0.4);
                            } else if (potentiometer.getVoltage() < armAngle) {

                                mU.setPower(0.4);
                            }
                        } else {

                            mU.setPower(0);
                            angleDone = true;

                        }
                        telemetry.addData("arm done", angleDone);

                        double reachTolerance = 50;
                        if ((Math.abs(mE.getCurrentPosition() - armReach) > reachTolerance) && angleDone) {
                            if (mE.getCurrentPosition() > armReach) {

                                mE.setPower(-0.65);
                            } else if (mE.getCurrentPosition() < armReach) {

                                mE.setPower(0.65);
                            }
                        } else {

                            mE.setPower(0);
                            extendDone = true;
                        }
                        telemetry.addData("extend done", extendDone);

                        if(angleDone && extendDone){

                            done = true;
                            changeStep();
                        }

                    break;

                    case "DRIVE_TO_LIMIT":
                        if(lAB.isPressed()) {  //uses limit switch to move arm to a known position
                            limitTriggered = true;
                            mE.setPower(0);
                            telescopePose = mE.getCurrentPosition();
                            done = true;
                            changeStep();
                        }

                        else if (!limitTriggered) {
                            mE.setPower(-0.65);
                        }
                        break;

//                    case "MOVE_ARM_OUT_OF_WAY":
//                        //raises arm so that it doesn't get in the way of the shooting rings
//                        armMotion = 1591;
//                        armStopPosition = armStartPosition - armMotion;
//
//                        if(mA.getCurrentPosition() <= armStopPosition) {
//                            mA.setPower(0);
//                            done = true;
//                            changeStep();
//                        }
//                        else {
//                            mA.setPower(-1);
//                        }
//                        break;

//                    case "READ_ARM_POSITION":
//                        //reads arm encoder position for future steps
//                        armStartPosition = mA.getCurrentPosition();
//                        done = true;
//                        break;

                    case "SPIN_SPINNER":
//                        if (!fifthCheck) {
//                            //if this step has not been run before, sets myTime to the runtime
//                            myTime = runtime.seconds();
//                            fifthCheck = true;
//                        }
                        double fifthTime = 4.5;

                        if ((runtime.seconds()) <= fifthTime ){
                            sSL.setPower(1);
                            sSR.setPower(-1);
                        }
                        if ((runtime.seconds()) > fifthTime) {
                            sSL.setPower(0);
                            sSR.setPower(0);
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

//                    case("MOVE_TO_BLUE_HUB"):
//                        double distanceThreshold = 2;
//                        double minDistance = Math.min(leftDistance, rightDistance);
//                        double targetDistance = 18 * 2.54;
//                        double toleranceDistance = 1 * 2.54;
//
//                        if (targetDistance - minDistance > toleranceDistance) {
//                            drive(0.4, 0, 0);
//
//                        } else if (targetDistance - minDistance < toleranceDistance) {
//                            drive(-0.4, 0, 0);
//
//
//                        } else {
//
//                            stopDriveTrain();
//                        }
//                        //done = true;
//                        break;

                    case "Test" :
                        //used for testing odometry
                        targetX = 0;
                        targetY = 12;
                        targetTheta = 0;
                        //done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;

                    case "WAIT":
                        //wait step that can be added in if necessary for a debug of the code
                        if(runtime.seconds() > 0 && runtime.seconds() < 5) {
                            done = false;
                        }
                        else {
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

                    case ("MOVE_FROM_WALL"):
                        //moves to way point based on the location of the target zone
                        targetDistanceX = 26;
                        targetDistanceY = 5;
                        done = (moveToLocation(targetDistanceX, targetDistanceY, 2, "rightDistance", "backDistance", 0, 5));
                        break;

                    case ("STRAFE_SQUARE"):
                        double strafeTolerance = 1;
                        double targetDistance3 = 24 * 2.54;
                        double toleranceDistance3 = 1 * 2.54;
                        //double backLeftDistance = rangeSensorBackLeft.getDistance(DistanceUnit.CM);
                        gyroZ = 0;

                        if (backDistance - strafeTolerance >= 1) {

                            drive(0, -0.8, 0);

                        } else if (targetDistance3 - backDistance > toleranceDistance3 && backDistance - strafeTolerance <= 1) {
                            drive(-0.4, 0, 0);

                        } else if (targetDistance3 - backDistance < toleranceDistance3) {
                            stopDriveTrain();
                            done = true;
                            changeStep();

                        }
                        //done = true;
                        break;

                    case ("DRIVE_TO_HUB"):
                        //moves to way point based on the location of the target zone
                        targetDistanceX = 10;
                        targetDistanceY = 44;
                        done = (moveToLocation(targetDistanceX, targetDistanceY, 1, "leftDistance", "backDistance", 90, 5));
                        break;

                    case "ROTATE_TO_90":
                        rotationAngle1 = 90;
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

                    case "POINT_TO_CARGO":
                        rotationAngle1 = 135;
                        tolerance = 5;
                        //totalAngleChange (second variable) cannot be 0
                        preciseRotationChange(rotationAngle1, 90);
                        if (Math.abs(gyroZ - rotationAngle1) < tolerance) {
                            done = true;
                            finalGyro = gyroZ;
                            changeStep();
                        }
                        break;

                    case ("DRIVE_TO_WAREHOUSE"):
                        //moves to way point based on the location of the target zone
                        targetDistanceX = 20;
                        targetDistanceY = 15;
                        targetTheta = 90;
                        done = (moveToLocation(targetDistanceX, targetDistanceY, 2, "leftDistance", "frontDistance", targetTheta, 5));
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

                    case ("DRIVE_FROM_WALL"):

                        if (!secondCheck) {
                            //if this step has not been run before, sets myTime to the runtime
                            myTime = runtime.seconds();
                            secondCheck = true;
                        }

                        double secondTime = 0.25;

                        if ((runtime.seconds() - myTime) <= secondTime) {
                            drive(-0.5, 0, 0);
                        }
                        if ((runtime.seconds() - myTime) > secondTime) {
                            done = true;
                            changeStep();
                        }
                        break;

                    case ("MOVE_TO_CAROUSEL"):
                        //moves to way point based on the location of the target zone
//                        targetDistanceX = 8;
//                        targetDistanceY = 5;
//                        done = (moveToLocation(targetDistanceX, targetDistanceY, 1, "leftDistance", "backDistance", 90, 5));

                        if (!thirdCheck) {
                            //if this step has not been run before, sets myTime to the runtime
                            myTime = runtime.seconds();
                            thirdCheck = true;
                        }

                        double thirdTime = 0.8;

                        if ((runtime.seconds() - myTime) <= thirdTime) {
                            drive(0.5, -0.3, 0);
                        }
                        if ((runtime.seconds() - myTime) > thirdTime) {
                            done = true;
                            changeStep();
                        }

                        break;

                    case ("MOVE_TO_CAROUSEL_2"):
                        //moves to way point based on the location of the target zone
//                        targetDistanceX = 8;
//                        targetDistanceY = 5;
//                        done = (moveToLocation(targetDistanceX, targetDistanceY, 1, "leftDistance", "backDistance", 90, 5));

                        if (!fourthCheck) {
                            //if this step has not been run before, sets myTime to the runtime
                            myTime = runtime.seconds();
                            fourthCheck = true;
                        }

                        double fourthTime = 1.5;

                        if ((runtime.seconds() - myTime) <= fourthTime) {
                            drive(0.1, 0.4, 0);
                        }
                        if ((runtime.seconds() - myTime) > fourthTime) {
                            drive(0,0,0);
                            done = true;
                            changeStep();
                        }

                        break;


//                    case ("MOVE_TO_BLUE_HUB"):
//                        //moves to way point based on the location of the target zone
//                        targetDistanceX = 27 / 2.54;
//                        targetDistanceY = 129 / 2.54;
//                        targetTheta = 90;
//                        done = (moveToLocation(targetDistanceX, targetDistanceY, 2, "leftDistance", "backRightDistance", targetTheta, 5));
//                        break;



                }
            }
        }
        shutdown();
    }

    public boolean moveToLocation
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
    }






