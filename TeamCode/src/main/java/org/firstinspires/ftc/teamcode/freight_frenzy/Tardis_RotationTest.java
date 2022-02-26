package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.ArrayList;

@Autonomous(name = "Tardis_RotationTest", group = "Autonomous")
@Disabled

public class Tardis_RotationTest extends AutoBase_FF {

    ArrayList<String> steps = new ArrayList<>();
    //creates list of steps to be completed

    double targetZoneX = 0;
    double targetZoneY = 0;
    double lastTheta = 0;
    String zoneName = "A";
    //initializes target zone variables, sets default target zone

    public void CreateSteps() {

        steps.add("ROTATE_LEFT");
        steps.add("SHORT_WAIT");
        steps.add("ROTATE_RIGHT");
        steps.add("WAIT");
        steps.add("STOP");

    }


    @Override
    public void runOpMode() {//Start of the initialization for autonomous

        boolean done = false;
        CreateSteps();

        //Init functions
        defineComponentsTestBed();

        double targetX = 0;
        double targetY = 0;
        double targetTheta = 0;
        double distanceTolerance = 2;
        double rotationTolerance = 1;
        double rotationAngle1 = 0;
        double rotationAngle2 = 0;
        double tolerance = 1;
        double initialGyro = gyroZ;
        double finalGyro = 0;

        boolean moveDone = false;
        double looseTolerance = 4;


//        //instance fields/global variables
//
//        ////////////////setting robot in initialization, readies robot for autonomous  /////////////
//        //counts number of rings using vision recognition program
//        pipeline = new CountRings.SkystoneDeterminationPipeline();
//        webcam.setPipeline(pipeline);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
//            }
//        });
//
//        //setting wrist and gripper to start position
//        sW.setPosition(1);
//        sG.setPosition(0.08);
//
//        while (!opModeIsActive() && !isStopRequested()){
//            //adds vision recognition telemetry for debug and check
//            telemetry.addData("Rings", CountRings.getCount());
//            telemetry.addData("Anti-Oranginess", pipeline.getAnalysis());
//            telemetry.update();
//            numberOfRings = CountRings.getCount();
//        }

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
                telemetry.addData("last theta", lastTheta);
                telemetry.addData("gyro", "" + String.format("%.2f deg", gyroZ));
                telemetry.addData("current position x", "" + String.format("%.2f in.", pose.x));
                telemetry.addData("current position y", "" + String.format("%.2f in.", pose.y));
                telemetry.addData("runtime two", runtimeTwo);
                telemetry.addData("initial gyro", initialGyro);
                telemetry.addData("final gyro", finalGyro);
                telemetry.addData("delta gyro", (finalGyro - initialGyro));
                telemetry.update();
                //Update global sensor values
                updatePoseStrafe();
                gyroUpdate();

                switch (currentStep) {

                    case "Test":
                        //used for testing odometry
                        targetX = 0;
                        targetY = 12;
                        targetTheta = 0;
                        done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;

                    case "ROTATE_LEFT":
                        rotationAngle1 = 10;
                        tolerance = 0.3;
                        //totalAngleChange (second variable) cannot be 0
                        preciseRotationChange(rotationAngle1, rotationAngle1);
                        if (Math.abs(gyroZ - rotationAngle1) < tolerance) {
                            done = true;
                            finalGyro = gyroZ;
                            changeStep();
                        }
                        break;

                    case "ROTATE_RIGHT":
                        rotationAngle2 = 0;
                        tolerance = 0.3;
                        //totalAngleChange (second variable) cannot be 0
                        preciseRotationChange(rotationAngle2, rotationAngle1);
                        if (Math.abs(gyroZ - rotationAngle2) < tolerance) {
                            done = true;
                            finalGyro = gyroZ;
                            changeStep();
                        }
                        break;

                    case "WAIT":
                        //wait step that can be added in if necessary for a debug of the code
                        if (runtime.seconds() > 0 && runtime.seconds() < 60) {
                            done = false;
                        } else {
                            done = true;
                            changeStep();
                        }
                        break;

                    case "SHORT_WAIT":
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





