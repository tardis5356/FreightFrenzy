package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Ginger_Tardis_Autonomous", group = "Autonomous")
@Disabled

public class Ginger_Tardis_Autonomous extends AutoBase_FF {

    ArrayList<String> steps = new ArrayList<>();
    //creates list of steps to be completed

    double targetZoneX = 0;
    double targetZoneY = 0;
    double lastTheta = 0;
    double myTime = 0;
    String elementPosition = "LEFT";
    //initializes target zone variables, sets default target zone

    public void CreateSteps() {
        steps.add("FIND_ELEMENT_POSITION");
//        steps.add("DRIVE_FROM_WALL");
//        steps.add("MOVE_FROM_WALL");
//        steps.add("ROTATE_TO_90");
//        steps.add("MOVE_TO_CAROUSEL");
       steps.add("WAIT");
//        steps.add("MOVE_TO_BLUE_HUB");
//        steps.add("ROTATE_TO_0");
//        steps.add("WAIT");
//        steps.add("ROTATE_TO_90");
        steps.add("WAIT");
//        steps.add("DRIVE_FORWARD");
//        steps.add("DRIVE_TO_WAREHOUSE");
        //steps.add("WAIT");
//        steps.add("POINT_AT_CARGO");
        steps.add("WAIT");
        steps.add("WAIT");
        steps.add("WAIT");
        steps.add("WAIT");
        steps.add("WAIT");

        steps.add("STOP");

    }


    @Override
    public void runOpMode() {//Start of the initialization for autonomous

        boolean done = false;
        CreateSteps();

        //Init functions
        defineComponentsGinger();

        double targetDistanceX = 0;
        double targetDistanceY = 0;
        double targetTheta = 0;


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


        //instance fields/global variables

//            ////////////////setting robot in initialization, readies robot for autonomous  /////////////
//            //find position of team element using vision recognition program
        pipeline = new TeamElementPositionTest.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        //setting wrist and gripper to start position
//            sW.setPosition(1);
//            sG.setPosition(0.08);


        while (!opModeIsActive() && !isStopRequested()) {
            //adds vision recognition telemetry for debug and check
            elementPosition = TeamElementPositionTest.getPosition();
            telemetry.addData("Team Element Position", elementPosition);
            telemetry.addData("Anti-Blueness", pipeline.getAnalysis());
            telemetry.update();
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
                telemetry.addData("left distance (in)", "" + String.format("%.2f", leftDistance / 2.54));
                telemetry.addData("right distance (in)", "" + String.format("%.2f", rightDistance / 2.54));
                //telemetry.addData("back left distance", "" + String.format("%.2f cm", backLeftDistance));
                telemetry.addData("back right distance (in)", "" + String.format("%.2f", backDistance / 2.54));
                telemetry.addData("front distance (in)", "" + String.format("%.2f", frontDistance / 2.54));


                telemetry.update();
                //Update global sensor values
                updatePoseStrafe();
                gyroUpdate();
                leftDistance = Range.clip(rangeSensorLeft.getDistance(DistanceUnit.CM), 0, 200);
                rightDistance = Range.clip(rangeSensorRight.getDistance(DistanceUnit.CM), 0, 200);
                backDistance = Range.clip(rangeSensorBack.getDistance(DistanceUnit.CM), 0, 200);
                frontDistance = Range.clip(rangeSensorFront.getDistance(DistanceUnit.CM), 0, 200);

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

                    case "FIND_ELEMENT_POSITION":
//                        //changes the target zone that the robot moves to based on the number of rings counted during initialization
//                        if (elementPosition == "LEFT") {
//                            elementPosition = "LEFT";
//                        } else if (elementPosition == "CENTER") {
//                            elementPosition = "CENTER";
//                        } else {
//                            elementPosition = "RIGHT";
//                        }
                       changeHubLevel(elementPosition);
                        done = true;
                        break;

                    case ("MOVE_FROM_WALL"):
                        //moves to way point based on the location of the target zone
                        targetDistanceX = 65 / 2.54;
                        targetDistanceY = 29 / 2.54;
                        done = (moveToLocation(targetDistanceX, targetDistanceY, 2, "rightDistance", "backRightDistance", 0, 5));
                        break;

                    case ("STRAFE_SQUARE"):
                        double strafeTolerance = 1;
                        double targetDistance3 = 24 * 2.54;
                        double toleranceDistance3 = 1 * 2.54;
                        //double backLeftDistance = rangeSensorBackLeft.getDistance(DistanceUnit.CM);
                        gyroZ = 0;

                        if (backDistance - strafeTolerance >= 1) {

                            drive(0, 0, -0.8);

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
                        targetDistanceX = 48;
                        targetDistanceY = 24;
                        done = (moveToLocation(targetDistanceX, targetDistanceY, 2, "rightDistance", "backRightDistance", 0, 5));
                        break;

                    case "ROTATE_TO_90":
                        rotationAngle1 = 90;
                        tolerance = 5;
                        //totalAngleChange (second variable) cannot be 0
                        preciseRotationChange(rotationAngle1, rotationAngle1);
                        if (Math.abs(gyroZ - rotationAngle1) < tolerance) {
                            done = true;
                            finalGyro = gyroZ;
                            changeStep();
                        }
                        break;

                    case "ROTATE_TO_0":
                        rotationAngle1 = 0;
                        tolerance = 5;
                        //totalAngleChange (second variable) cannot be 0
                        preciseRotationChange(rotationAngle1, 90);
                        if (Math.abs(gyroZ - rotationAngle1) < tolerance) {
                            done = true;
                            finalGyro = gyroZ;
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
                        targetDistanceX = 38 / 2.54;
                        targetDistanceY = 39 / 2.54;
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
                        }
                        break;

                    case ("MOVE_TO_CAROUSEL"):
                        //moves to way point based on the location of the target zone
                        targetDistanceX = 26 / 2.54;
                        targetDistanceY = 15 / 2.54;
                        done = (moveToLocation(targetDistanceX, targetDistanceY, 2, "leftDistance", "backRightDistance", 90, 5));
                        break;

                    case ("MOVE_TO_BLUE_HUB"):
                        //moves to way point based on the location of the target zone
                        targetDistanceX = 27 / 2.54;
                        targetDistanceY = 129 / 2.54;
                        targetTheta = 90;
                        done = (moveToLocation(targetDistanceX, targetDistanceY, 2, "leftDistance", "backRightDistance", targetTheta, 5));
                        break;


                    case "WAIT":
                        //wait step that can be added in if necessary for a debug of the code
                        if (runtime.seconds() > 0 && runtime.seconds() < 2) {
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
                        break;

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

    public void changeHubLevel(String elementPosition) {
        //sets coordinates for all three target zones, target zone is chosen depending on number of rings counted
        switch (elementPosition) {

            case "LEFT":
//                    targetZoneX = 26;
//                    targetZoneY = 68;
                break;

            case "CENTER":
//                    targetZoneX = 2;
//                    targetZoneY = 88;
                break;

            case "RIGHT":
//                    targetZoneX = 26;
//                    targetZoneY = 112;
                break;

            default:
//                    targetZoneX = 0;
//                    targetZoneY = 0;
                break;

        }
    }
}






