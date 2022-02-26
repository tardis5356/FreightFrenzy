package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@Autonomous(name = "Fred_Odometry_Test", group = "Autonomous")

public class Fred_Odometry_Test extends AutoBase_FF {

    ArrayList<String> steps = new ArrayList<>();
    //creates list of steps to be completed

    double targetZoneX = 0;
    double targetZoneY = 0;
    double lastTheta = 0;
    double myTime = 0;

    //initializes target zone variables, sets default target zone

    public void CreateSteps() {
        //do not rotate inside of moveToPose()

//        steps.add("STRAFE_TO_CAROUSEL");
//        steps.add("DRIVE_INTO_CAROUSEL");
//        steps.add("SPIN_SPINNER");
//        steps.add("DRIVE_FORWARD");
//        steps.add("ROTATE_TO_90");
//        steps.add("MOVE_TO_HUB");
//        steps.add("HELLO_WORLD");
//        steps.add("MOVE_SERVOS");
//        steps.add("MOVE_X_SERVO");
        steps.add("TEST_2");
        //steps.add("ROTATE_TO_90");
        steps.add("TEST");
        steps.add("WAIT");
        steps.add("STOP");


    }

//hello world :)
    //the mitochondria is the powerhouse of the cell
    //gray's chemistry class is no bueno
    //ziti is the best
    //who is craig? what does he look like? does he exist?
    //why are we here


    @Override
    public void runOpMode() {//Start of the initialization for autonomous

        boolean done = false;
        CreateSteps();

        //Init functions
        defineComponentsFred();
        updatePoseStrafe();

        double targetDistanceX = 0;
        double targetDistanceY = 0;
        double targetTheta = 0;
        String elementPosition = "NONE";

        double distanceTolerance = 5;
        double rotationTolerance = 5;
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
//        sI.setPower(0);
        //double telescopePose = 0;

        //instance fields/global variables

//            ////////////////setting robot in initialization, readies robot for autonomous  /////////////
//        pipeline = new TeamElementPositionTest.SkystoneDeterminationPipeline();
//        webcam.setPipeline(pipeline);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                //Upright rotation works, do not set to sideways left
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });

//        while (!opModeIsActive() && !isStopRequested()) {
//            //adds vision recognition telemetry for debug and check
//            elementPosition = TeamElementPositionTest.getPosition();
//            telemetry.addData("Team Element Position", elementPosition);
//            telemetry.addData("Hub level", hubLevel);
//            telemetry.addData("Anti-Blueness", pipeline.getAnalysis());
//            telemetry.update();
//            //sets arm, extension, and wrist on initialization to get within 18 inches
//            scrunchUpBot();
//
//            if (elementPosition == "LEFT") {
//                hubLevel = "BOTTOM";
//            } else if (elementPosition == "CENTER") {
//                hubLevel = "MIDDLE";
//            } else if (elementPosition == "RIGHT") {
//                hubLevel = "TOP";
//            }
//            changeHubLevel(elementPosition);
//        }

        lowerOdometerServos();

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
//                telemetry.addData("sX position", sX.getPosition());
//                telemetry.addData("sYL position", sYL.getPosition());
//                telemetry.addData("sYR position", sYR.getPosition());
                telemetry.addData("gyro", gyroZ);
                telemetry.addData("x pos", pose.x);
                telemetry.addData("y pos", pose.y);
                telemetry.addData("forward", forwardPower);
                telemetry.addData("strafe", strafePower);
                telemetry.addData("rotate", rotatePower);
                telemetry.addData("errInX", errInX);
                telemetry.addData("errInY", errInY);
                telemetry.update();

//                fredTelemetry();

                //Update global sensor values
                updatePoseStrafe();
                gyroUpdate();

//                readDistanceSensors();
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

                    case "STOP":
                        //stops drive train
                        drive(0, 0, 0);
                        done = true;
                        changeStep();
                        break;

//                    case "HELLO_WORLD":
//                        telemetry.addLine("Hello World");
//                        telemetry.update();
//                        if (runtime.seconds() > 0 && runtime.seconds() < 5) {
//                            done = false;
//                        } else {
//                            done = true;
//                            changeStep();
//                        }
//                        break;

                    case "DRIVE_FORWARD":
                        targetX = 0;
                        targetY = 3;
                        targetTheta = 0;
                        done = (moveToLocationOdometry(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
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

                    case "STRAFE_TO_CAROUSEL":
                        targetX = -26;
                        targetY = 24;
                        targetTheta = 0;
                        done = (moveToLocationOdometry(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;

                    case "SPIN_SPINNER":
                        sSL.setPower(-1);
                        if (runtime.seconds() > 0 && runtime.seconds() < 4) {
                            done = false;
                        } else {
                            sSL.setPower(0);
                            done = true;
                            changeStep();
                        }
                        break;

                    case "DRIVE_INTO_CAROUSEL":
                        drive(0.5, 0, 0);
                        if (runtime.seconds() > 0 && runtime.seconds() < 1) {
                            done = false;
                        } else {
                            done = true;
                            changeStep();
                        }
                        break;

                    case "MOVE_TO_HUB":
                        targetX = 24;
                        targetY = 10;
                        targetTheta = -90;
                        done = (moveToLocationOdometry(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;

//                    case "FIND_ELEMENT_POSITION":
//                        //changes the target zone that the robot moves to based on the number of rings counted during initialization
//                        changeHubLevel(hubLevel);
//                        done = true;
//                        changeStep();
//                        break;
                    case "TEST":
                        //used for testing odometry
                        targetX = 24;
                        targetY = 24;
                        targetTheta = 0;
                        done = (moveToLocationOdometry(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;

                    case "TEST_2":
                        //used for testing odometry
                        targetX = 0;
                        targetY = 0;
                        targetTheta = 90;
                        done = (moveToLocationOdometry(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;

                    case "MOVE_YL_SERVO":
                        sYL.setPosition(0);
                        if (runtime.seconds() > 0 && runtime.seconds() < 5) {
                            done = false;
                        } else {
                            done = true;
                            changeStep();
                        }
                        break;

                    case "MOVE_SERVOS":
                        sYL.setPosition(0);
                        if (runtime.seconds() > 2 && runtime.seconds() < 4) {
                            sYR.setPosition(1);
                        } else if (runtime.seconds() >= 4 && runtime.seconds() < 6) {
                            sX.setPosition(1);
                        } else if (runtime.seconds() >= 6) {
                            done = true;
                            changeStep();
                        }
                        break;

                    case "MOVE_YR_SERVO":
                        sYR.setPosition(1);
                        if (runtime.seconds() > 0 && runtime.seconds() < 5) {
                            done = false;
                        } else {
                            done = true;
                            changeStep();
                        }
                        break;

                    case "MOVE_X_SERVO":
                        sX.setPosition(1);
                        if (runtime.seconds() > 0 && runtime.seconds() < 5) {
                            done = false;
                        } else {
                            done = true;
                            changeStep();
                        }
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

                }
            }
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
            moveToPose(targetX, targetY, targetTheta, 100);
            return false;
        }

    }
}
