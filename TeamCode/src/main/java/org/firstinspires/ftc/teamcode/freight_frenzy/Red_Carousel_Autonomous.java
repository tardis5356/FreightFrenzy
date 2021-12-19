package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Red_Carousel_Autonomous", group = "Autonomous")

public class Red_Carousel_Autonomous extends AutoBase_FF {

    /*not all code below is used in our current autonomous, but can be added to the autonomous
    functionality later when sufficient testing has been done*/

    ArrayList<String> steps = new ArrayList<>();
    //creates list of steps to be completed

    //initialization variables
    double targetZoneX = 0;
    double targetZoneY = 0;
    double lastTheta = 0;
    double myTime = 0;
    String hubLevel = "BOTTOM";
    double armAngle = 0;
    double armReach = 0;
    double telescopePose = 0;

    public void CreateSteps() {

        steps.add("DRIVE_FROM_WALL");
        steps.add("MOVE_FROM_WALL");
        steps.add("ROTATE_TO_90");
        steps.add("MOVE_TO_CAROUSEL");
        steps.add("MOVE_TO_CAROUSEL_2");
        steps.add("SPIN_SPINNER");
        steps.add("PARK_IN_STORAGE_UNIT");
        steps.add("STOP");


    }


    @Override
    public void runOpMode() {//Start of the initialization for autonomous

        boolean done = false;
        CreateSteps();

        //Init functions
        defineComponentsFred();

        //instance fields/global variables
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

//            ////////////////setting robot in initialization, readies robot for autonomous  /////////////

        //initializes vision recognition code
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

        //sets bot to initial position for autonomous, fits within 18 inches
        scrunchUpBot();




        while (!opModeIsActive() && !isStopRequested()) {
            //adds vision recognition telemetry for debug and check
            elementPosition = TeamElementPositionTest.getPosition();
            telemetry.addData("Team Element Position", elementPosition);
            telemetry.addData("Hub level", hubLevel);
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

                //telemetry so we know what the robot is doing
                telemetry.addData("current step", currentStep);
                telemetry.addData("gyro", "" + String.format("%.2f deg", gyroZ));

                //distance sensor and potentiometer readings
                telemetry.addData("left distance (in)", "" + String.format("%.2f", leftDistance));
                telemetry.addData("right distance (in)", "" + String.format("%.2f", rightDistance));
                telemetry.addData("back distance (in)", "" + String.format("%.2f", backDistance));
                telemetry.addData("front distance (in)", "" + String.format("%.2f", frontDistance));
                telemetry.addData("potentiometer angle", potentiometer.getVoltage());
                telemetry.update();
                //Update global sensor values
                updatePoseStrafe();
                gyroUpdate();

                //clips range sensors at 0 and 200 to eliminate unreliable readings
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

                    case "PARK_IN_STORAGE_UNIT":
                        //parks robot in storage unit
                        targetDistanceX = 25;
                        targetDistanceY = 1;
                        done = (moveToLocation(targetDistanceX, targetDistanceY, 2, "rightDistance", "backDistance", -90, 5));
                        break;

                    case "MOVE_ARM":
                        //moves arm to a specified potentiometer setting
                        double potTolerance = 0.1;
                        if (Math.abs(potentiometer.getVoltage() - armAngle) > potTolerance) {
                            if (potentiometer.getVoltage() > armAngle) {

                                mU.setPower(-1);
                            } else if (potentiometer.getVoltage() < armAngle) {

                                mU.setPower(1);
                            }
                        } else {

                            mU.setPower(0);
                            done = true;
                        }

                        double reachTolerance = 10;
                        if (Math.abs(mE.getCurrentPosition() - armReach) > reachTolerance) {
                            if (mE.getCurrentPosition() > armReach) {

                                mE.setPower(-1);
                            } else if (mE.getCurrentPosition() < armReach) {

                                mE.setPower(1);
                            }
                        } else {

                            mE.setPower(0);
                            done = true;
                        }
                        break;

                    case "DRIVE_TO_LIMIT":
                        if(lAB.isPressed()) {  //uses limit switch to move arm to a known position
                            limitTriggered = true;
                            telescopePose = mE.getCurrentPosition();
                        }

                        else if (!limitTriggered) {
                            mE.setPower(-1);
                        }
                        done = true;
                        break;


                    case "SPIN_SPINNER":
                        //runs spinner servo for a specific amount of time
                        if (!fifthCheck) {
                            //if this step has not been run before, sets myTime to the runtime
                            myTime = runtime.seconds();
                            fifthCheck = true;
                        }
                        double fifthTime = 8;

                        if ((runtime.seconds() - myTime) <= fifthTime ){
                            sSL.setPower(1);
                            sSR.setPower(-1);
                        }
                        if ((runtime.seconds() - myTime) > fifthTime) {
                            sSL.setPower(0);
                            sSR.setPower(0);
                            done = true;
                            changeStep();
                        }
                        break;


                    case "FIND_ELEMENT_POSITION":
                        //changes the target zone that the robot moves to based on the number of rings counted during initialization
                        if (elementPosition == "LEFT") {
                            hubLevel = "BOTTOM";
                        } else if (elementPosition == "CENTER") {
                            hubLevel = "CENTER";
                        } else {
                            hubLevel = "TOP";
                        }
                        changeHubLevel(elementPosition);
                        done = true;
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
                        break;

                    case ("MOVE_FROM_WALL"):
                        //moves to way point based on the location of the target zone
                        targetDistanceX = 26;
                        targetDistanceY = 5;
                        done = (moveToLocation(targetDistanceX, targetDistanceY, 2, "leftDistance", "backDistance", 0, 5));
                        break;

                    case ("DRIVE_TO_HUB"):
                        //drives to blue alliance hub
                        targetDistanceX = 12;
                        targetDistanceY = 48;
                        done = (moveToLocation(targetDistanceX, targetDistanceY, 2, "leftDistance", "backDistance", 90, 5));
                        break;

                    case "ROTATE_TO_90":
                        //rotates robot to 90 degrees
                        rotationAngle1 = -90;
                        tolerance = 5;
                        //totalAngleChange (second variable) cannot be 0
                        rotateSigmoid(rotationAngle1);
                        if (Math.abs(gyroZ - rotationAngle1) < tolerance) {
                            done = true;
                            //finalGyro = gyroZ;
                            changeStep();
                        }
                        break;

                    case "ROTATE_TO_0":
                        //rotates robot to 0 degrees
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
                        //points robot to ideal location for picking up cargo in tele
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
                        //drives forward for a short amount of time
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
                        //drives robot a short distance from wall, based on time
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
                     //drives to a rough position near the carousel
                        if (!thirdCheck) {
                            //if this step has not been run before, sets myTime to the runtime
                            myTime = runtime.seconds();
                            thirdCheck = true;
                        }

                        double thirdTime = 0.8;

                        if ((runtime.seconds() - myTime) <= thirdTime) {
                            drive(0.5, -0.1, 0);
                        }
                        if ((runtime.seconds() - myTime) > thirdTime) {
                            done = true;
                        }

                        break;

                    case ("MOVE_TO_CAROUSEL_2"):
                        //moves to a more precise position, with spinner against carousel
                        if (!fourthCheck) {
                            //if this step has not been run before, sets myTime to the runtime
                            myTime = runtime.seconds();
                            fourthCheck = true;
                        }

                        double fourthTime = 0.5;

                        if ((runtime.seconds() - myTime) <= fourthTime) {
                            drive(0.1, -0.4, 0);
                        }
                        if ((runtime.seconds() - myTime) > fourthTime) {
                            drive(0,0,0);
                            done = true;
                        }

                        break;
                }
            }
        }
        //stops robot
        shutdown();
    }

    public boolean moveToLocation
        //function for setting a point with distance, changes step when robot position is in a certain tolerance
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
    public void changeHubLevel(String hubLevel) {
        //sets arm position for all three hub levels, hub level is chosen depending on position of team element
        switch (hubLevel) {

            case "BOTTOM":

                armAngle = 3.3;
                armReach = 0;
                break;

            case "CENTER":

                armAngle = 2.6;
                armReach = 0;
                break;

            case "TOP":

                armAngle = 1.7;
                armReach = telescopePose + 100;
                break;

            default:

                armAngle = 1.7;
                armReach = telescopePose + 100;
                break;

        }
    }
}






