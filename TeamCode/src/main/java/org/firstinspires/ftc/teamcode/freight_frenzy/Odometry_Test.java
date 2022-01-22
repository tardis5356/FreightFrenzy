package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Odometry_Test", group = "Autonomous")

public class Odometry_Test extends AutoBase_FF {

    ArrayList<String> steps = new ArrayList<>();
    //creates list of steps to be completed

    double targetZoneX = 0;
    double targetZoneY = 0;
    double lastTheta = 0;
    double myTime = 0;

    //initializes target zone variables, sets default target zone

    public void CreateSteps() {


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

                //Update global sensor values
                updatePoseStrafe();
                gyroUpdate();

                readDistanceSensors();
                switch (currentStep) {

                    case "STOP":
                        //stops drive train
                        drive(0, 0, 0);
                        done = true;
                        changeStep();
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

                }
            }
        }
    }
}
