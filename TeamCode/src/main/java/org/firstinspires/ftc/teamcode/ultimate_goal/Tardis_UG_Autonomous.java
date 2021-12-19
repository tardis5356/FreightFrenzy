package org.firstinspires.ftc.teamcode.ultimate_goal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Tardis_UG_Autonomous", group = "Autonomous")
@Disabled

public class Tardis_UG_Autonomous extends AutoBaseTB1 {

    ArrayList<String> steps = new ArrayList<>();
    //creates list of steps to be completed

    double targetZoneX = 0;
    double targetZoneY = 0;
    double lastTheta = 0;
    String zoneName = "A";
    //initializes target zone variables, sets default target zone

    public void CreateSteps() {

       /*      steps.add("LOCATION_1");
             steps.add("LOCATION_2");
        steps.add("LOCATION_1");
        steps.add("LOCATION_2");
        steps.add("LOCATION_1");
        steps.add("LOCATION_2");
        steps.add("LOCATION_1");
        steps.add("LOCATION_2");
        steps.add("LOCATION_1");
        steps.add("LOCATION_2");
        steps.add("LOCATION_1");
        steps.add("LOCATION_2");*/
           steps.add("COUNT_RINGS");
           //steps.add("MOVE_AWAY_FROM_WALL"); //don't need
           steps.add("READ_ARM_POSITION");
           steps.add("MOVE_ARM_OUT_OF_WAY");
           steps.add("WAYPOINT_1");
           steps.add("PS1");
           //steps.add("WAIT"); //don't need
           //steps.add("ROTATE_PS1"); //don't need
           steps.add("SHOOT_RING");
           //steps.add("WAYPOINT_2"); //don't need
           steps.add("PS2");
           //steps.add("WAIT"); //don't need
           steps.add("SHOOT_RING");
           //steps.add("WAYPOINT_3"); //don't need
           steps.add("PS3");
           steps.add("SHOOT_RING");
           //steps.add("IDLE_SHOOTER"); //don't need
           steps.add("DRIVE_TO_TZ_WAYPOINT");
           steps.add("DRIVE_TO_TZ");
           steps.add("READ_ARM_POSITION");
           steps.add("DELIVER_WOBBLE");
           //steps.add("GO_TO_LAUNCH_LINE"); //don't need
           //steps.add("BACK_FROM_WOBBLE"); //don't need
           steps.add("2ND_WOBBLE_WAYPOINT");
           steps.add("2ND_WOBBLE");
           steps.add("GRAB_2ND_WOBBLE");
           steps.add("DRIVE_TO_TZ2");
           steps.add("DROP_2ND_WOBBLE");
           //steps.add("BACK_FROM_WOBBLE2"); //don't need
           steps.add("SET_FOR_TELE");
           steps.add("STOP");
    }


    @Override
    public void runOpMode() {//Start of the initialization for autonomous

        boolean done = false;
        CreateSteps();

        //Init functions
        defineComponents();

        double targetX = 0;
        double targetY = 0;
        double targetTheta = 0;
        double powerShotDistanceTolerance = 2;
        double powerShotRotationTolerance = 0.5;
        double distanceTolerance = 2;
        double rotationTolerance = 1;
        double endDistanceTolerance = 1;
        double endRotationTolerance = 0.5;
        int armStartPosition =  mA.getCurrentPosition();
        int armStopPosition;
        int numberOfRings = -1;
        double rotationAngle = 0;
        int armMotion = 0;
        boolean limitTriggered = false;
        double tolerance = 1;
        boolean moveDone = false;
        double looseTolerance = 4;
        boolean firstCheck = false;
        double firstTime = 0;

        //instance fields/global variables

        ////////////////setting robot in initialization, readies robot for autonomous  /////////////
        //counts number of rings using vision recognition program
        pipeline = new CountRings.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        //setting wrist and gripper to start position
        sW.setPosition(1);
        sG.setPosition(0.08);

        while (!opModeIsActive() && !isStopRequested()){
            //adds vision recognition telemetry for debug and check
            telemetry.addData("Rings", CountRings.getCount());
            telemetry.addData("Anti-Oranginess", pipeline.getAnalysis());
            telemetry.update();
            numberOfRings = CountRings.getCount();
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
                telemetry.addData("last theta", lastTheta);
                telemetry.addData("potentiometer angle", getElevAngle(potentiometer.getVoltage()));
                telemetry.addData("gyro", "" + String.format("%.2f deg", gyroZ));
                telemetry.addData("current position x", "" + String.format("%.2f in.", pose.x));
                telemetry.addData("current position y", "" + String.format("%.2f in.", pose.y));
                telemetry.addData("arm position", mA.getCurrentPosition());
                telemetry.addData("wrist position", sW.getPosition());
                telemetry.addData("gripper position", sG.getPosition());
                telemetry.addData("runtime two", runtimeTwo);
                telemetry.update();
                //Update global sensor values
                updatePoseStrafe();
                gyroUpdate();

                switch (currentStep) {

                    case "Test" :
                        //used for testing odometry
                        targetX = 0;
                        targetY = 12;
                        targetTheta = 0;
                        done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;

                    case "LOCATION_1" :
                        //used for testing odometry
                        targetX = 0;
                        targetY = 0;
                        targetTheta = 0;
                        telemetry.addData("firstCheck", firstCheck);
                        telemetry.addData("firstTime", firstTime);
                        telemetry.addData("runtime seconds", runtime.seconds());
                        if(!firstCheck) {
                            firstCheck = moveToLocation(targetX, targetY, targetTheta, powerShotDistanceTolerance, powerShotRotationTolerance);
                        }
                        if(firstCheck && firstTime == 0){
                            firstTime = runtime.seconds();
                            //stops all drive motors to ensure bot does not move location
                            drive(0,0,0);
                        }
                        if(runtime.seconds() - firstTime > 0.5){
                            if(done){
                                firstCheck = false;
                                firstTime = 0;
                                //stops all drive motors to ensure bot does not move location
                                drive(0,0,0);
                            }
                            done = moveToLocation(targetX, targetY, targetTheta, powerShotDistanceTolerance, powerShotRotationTolerance);
                        }
                        break;

                    case "LOCATION_2" :
                        //used for testing odometry
                        targetX = 0;
                        targetY = 48;
                        targetTheta = 0;
                        telemetry.addData("firstCheck", firstCheck);
                        telemetry.addData("firstTime", firstTime);
                        telemetry.addData("runtime seconds", runtime.seconds());
                        if(!firstCheck) {
                            firstCheck = moveToLocation(targetX, targetY, targetTheta, powerShotDistanceTolerance, powerShotRotationTolerance);
                        }
                        if(firstCheck && firstTime == 0){
                            firstTime = runtime.seconds();
                            //stops all drive motors to ensure bot does not move location
                            drive(0,0,0);
                        }
                        if(runtime.seconds() - firstTime > 0.5){
                            if(done){
                                firstCheck = false;
                                firstTime = 0;
                                //stops all drive motors to ensure bot does not move location
                                drive(0,0,0);
                            }
                            done = moveToLocation(targetX, targetY, targetTheta, powerShotDistanceTolerance, powerShotRotationTolerance);
                        }
                        break;

                    case "GRAB_2ND_WOBBLE":
                        //uses wrist and gripper to grab 2nd wobble
                        sG.setPosition(0.08);
                        if(runtime.seconds() > 0.5) {
                            sW.setPosition(0.1);
                        }
                        if(runtime.seconds() > 1) {
                            done = true;
                        }
                        break;
                    case "DROP_2ND_WOBBLE":
                        //drops 2nd wobble
                        sG.setPosition(0.25);
                        if(runtime.seconds() > 0.5) {
                            done = true;
                        }
                        break;

                    case "2ND_WOBBLE":
                        //drives to approximate position of 2nd wobble; deploys arm to position for grabbing wobble
                        targetX = 28;
                        targetY = 27;
                        targetTheta = -180;
                        done = (moveToLocation(targetX, targetY, targetTheta, 1.5, 1));

                        if (frontLimit.isPressed()) {
                            mA.setPower(0);
                        } else {
                            mA.setPower(-1);
                        }

                        break;

                    case "2ND_WOBBLE_WAYPOINT":
                        //moves to waypoint before delivering 2nd wobble
                        sW.setPosition(0.36);
                        targetX = 28;
                        targetY = 41;
                        targetTheta = -179; //
                        done = moveToLocation(targetX, targetY, targetTheta, looseTolerance, rotationTolerance);
                        if(runtime.seconds() > 2) {
                            if (frontLimit.isPressed()) {
                                mA.setPower(0);
                            } else {
                                mA.setPower(-1);
                            }
                        }

                        break;

                    case "DRIVE_TO_TZ2":
                        //moves to target zone
                        targetX = targetZoneX-3;
                        targetY = targetZoneY;
                        targetTheta = -90;
                        done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;

                    case "BACK_FROM_WOBBLE2":
                        //moves robot away from target zone in x, avoids robot hitting 2nd wobble when it drives to launch line
                        sG.setPosition(0.1);
                        targetX = targetZoneX-12;
                        targetY = targetZoneY;
                        targetTheta = -90;
                        done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;


                    case "MOVE_AWAY_FROM_WALL":
                        //moves robot a short distance from wall to avoid the elevator contacting the wall
                        targetX = 0;
                        targetY = 5;
                        targetTheta = 0;
                        done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;

                    case "LOWER_INTAKE":
                        //lowers the intake ramp to prepare for the intake of rings
                        sLower.setPosition(1);
                        done = true;
                        changeStep();
                        break;

                    case "MOVE_ARM_OUT_OF_WAY":
                        //raises arm so that it doesn't get in the way of the shooting rings
                        armMotion = 1591;
                        armStopPosition = armStartPosition - armMotion;

                        if(mA.getCurrentPosition() <= armStopPosition) {
                            mA.setPower(0);
                            done = true;
                            changeStep();
                        }
                        else {
                            mA.setPower(-1);
                        }
                        break;

                    case "COUNT_RINGS":
                        //changes the target zone that the robot moves to based on the number of rings counted during initialization
                        if (numberOfRings == 0) {
                            zoneName = "A";
                        } else if (numberOfRings == 1) {
                            zoneName = "B";
                        } else {
                            zoneName = "C";
                        }
                        changeTargetZone(zoneName);
                        done = true;
                        break;

                    case "IDLE_SHOOTER":
                        //idles shooter motor after robot is done shooting
                        mS.setPower(0.25);
                        if(runtime.seconds() > 1) {
                            done = true;
                            changeStep();
                        }
                        break;

                    case "ROTATE_PS1":
                        /////////////////////////this step is not currently used\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
                        //rotates to power shot 1
                        targetX = 0;
                        targetY = 55;
                        targetTheta = 0;
                        done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        /*rotationAngle = 60;
                        tolerance = 1;
                        gyroAdjust(-0.5, rotationAngle);
                        if (Math.abs(gyroZ - rotationAngle) < tolerance) {
                            done = true;
                            changeStep();
                        }
                        */
                       break;
                    case "SHOOT_RING":
                        //shoots rings for power shots
                        boolean doneShooting = false;
                        //stops all drive motors to ensure bot does not move location
                        drive(0,0,0);
                        if(runtime.seconds() < 1) {
                            sP.setPosition(1);//shooting .6`5
                        }
                        else if(runtime.seconds() > 1 && runtime.seconds() < 1.8) {
                            sP.setPosition(0);//not shooting .515
                        }
                        else if(runtime.seconds() > 1.8){
                            //changes step
                            done = true;
                            changeStep();
                        }
                        break;

                    case "DELIVER_WOBBLE":
                        // drops wobble in target zone
                        armMotion = 2500;
                        armStopPosition = armStartPosition - armMotion;
                        double delayTime = 1;
                        if(mA.getCurrentPosition() <= armStopPosition) {

                            mA.setPower(0);

                            if(runtime.seconds() > delayTime && runtime.seconds() < (delayTime + 0.5)) {
                                sG.setPosition(0.25);
                                mA.setPower(1);
                            }
//                            if(runtime.seconds() > (delayTime + 0.5) && runtime.seconds() < (delayTime + 0.8)) { }
                            if(runtime.seconds() > (delayTime + 0.5)) {
                                done = true;
                                changeStep();
                            }
                        }
                        else {
                            mA.setPower(-1);
                            sW.setPosition(0.65);
                        }
                        break;

                    case "BACK_FROM_WOBBLE":
                        //moves robot away from target zone in x, avoids robot hitting wobble when it drives to launch line
                        targetX = targetZoneX - 12;
                        targetY = targetZoneY;
                        targetTheta = -90;
                        done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;


                    case "DRIVE_TO_TZ":
                        //moves to target zone
                        targetX = targetZoneX;
                        targetY = targetZoneY;
                        targetTheta = -90;
                        done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;

                    case "DRIVE_TO_TZ_WAYPOINT":
                        //moves to way point based on the location of the target zone
                        mS.setPower(0.25);
                        targetX = targetZoneX - 24;
                        targetY = targetZoneY;
                        targetTheta = -90;
                        done = (moveToLocation(targetX, targetY, targetTheta, looseTolerance, rotationTolerance));
                        break;

                    case "WAYPOINT_1":
                        //moves to approximate shooting position quickly
                        int idealElevAngle = 24;

                        targetX = -15;
                        targetY = 50;
                        targetTheta = 0;

                        //added for testing purposes
                        //done = (moveToLocation(targetX, targetY, targetTheta, powerShotDistanceTolerance, powerShotRotationTolerance));

                        //removed for testing purposes
                        //goToAngle(idealElevAngle);


                        mS.setPower(1); // power up shooter

                        if(getElevAngle(potentiometer.getVoltage()) > idealElevAngle - 0.5 && getElevAngle(potentiometer.getVoltage()) < idealElevAngle + 0.5) {
                            done = true;
                            mE.setPower(0);
                        }
                        else {
                            moveToLocation(targetX, targetY, targetTheta, looseTolerance, rotationTolerance);
                            goToAngle(idealElevAngle);
                        }

                        //done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                        break;

                    case "WAYPOINT_2":
                        targetX = -8;
                        targetY = 50;
                        targetTheta = 0;
                        done = (moveToLocation(targetX, targetY, targetTheta, looseTolerance, rotationTolerance));
                        break;

                    case "WAYPOINT_3":
                        targetX = 2;
                        targetY = 50;
                        targetTheta = 0;
                        done = (moveToLocation(targetX, targetY, targetTheta, looseTolerance, rotationTolerance));
                        break;

                    case "PS1":
                        //movement is smaller, allowing for more precision
                        //goes to a known location to shoot power shots from, moves elevator to shooting position
                        //int idealElevAngle = 25;
                        targetX = -13;
                        targetY = 60;
                        targetTheta = 0;
                        /*telemetry.addData("firstCheck", firstCheck);
                        telemetry.addData("firstTime", firstTime);
                        telemetry.addData("runtime seconds", runtime.seconds());
                        if(!firstCheck) {
                            firstCheck = moveToLocation(targetX, targetY, targetTheta, powerShotDistanceTolerance, powerShotRotationTolerance);
                        }
                        if(firstCheck && firstTime == 0){
                            firstTime = runtime.seconds();
                            //stops all drive motors to ensure bot does not move location
                            drive(0,0,0);
                        }
                        if(runtime.seconds() - firstTime > 0.2){
                            if(done){
                                firstCheck = false;
                                firstTime = 0;
                                //stops all drive motors to ensure bot does not move location
                                drive(0,0,0);
                            }
                            done = moveToLocation(targetX, targetY, targetTheta, powerShotDistanceTolerance, powerShotRotationTolerance);
                        }*/

                        //added for testing purposes
                        done = (moveToLocation(targetX, targetY, targetTheta, powerShotDistanceTolerance, powerShotRotationTolerance));

                        //removed for testing purposes
                        /*goToAngle(idealElevAngle);
                        //mS.setPower(1);

                        moveToLocation(targetX, targetY, targetTheta, powerShotDistanceTolerance,powerShotRotationTolerance);
                        if(getElevAngle(potentiometer.getVoltage()) > idealElevAngle - 0.5 && getElevAngle(potentiometer.getVoltage()) < idealElevAngle + 0.5){
                            done = true;
                        }
                        if(done){
                            mE.setPower(0);
                        }*/
                        break;

                    case "PS2":
                        targetX = -3;
                        targetY = 60;
                        targetTheta = 0;
                        /*telemetry.addData("firstCheck", firstCheck);
                        telemetry.addData("firstTime", firstTime);
                        telemetry.addData("runtime seconds", runtime.seconds());
                        if(!firstCheck) {
                            firstCheck = moveToLocation(targetX, targetY, targetTheta, powerShotDistanceTolerance, powerShotRotationTolerance);
                        }
                        if(firstCheck && firstTime == 0){
                            firstTime = runtime.seconds();
                            //stops all drive motors to ensure bot does not move location
                            drive(0,0,0);
                        }
                        if(runtime.seconds() - firstTime > 0.2){
                            if(done){
                                firstCheck = false;
                                firstTime = 0;
                                //stops all drive motors to ensure bot does not move location
                                drive(0,0,0);
                            }
                            done = moveToLocation(targetX, targetY, targetTheta, powerShotDistanceTolerance, powerShotRotationTolerance);
                        }*/
                        //moves to power shot 2
                        done = moveToLocation(targetX, targetY, targetTheta, powerShotDistanceTolerance, powerShotRotationTolerance);
                        break;

                    case "PS3":
                        targetX = 4;
                        targetY = 60;
                        targetTheta = 0;
                        /*telemetry.addData("firstCheck", firstCheck);
                        telemetry.addData("firstTime", firstTime);
                        telemetry.addData("runtime seconds", runtime.seconds());
                        if(!firstCheck) {
                            firstCheck = moveToLocation(targetX, targetY, targetTheta, powerShotDistanceTolerance, powerShotRotationTolerance);
                        }
                        if(firstCheck && firstTime == 0){
                            firstTime = runtime.seconds();
                            //stops all drive motors to ensure bot does not move location
                            drive(0,0,0);
                        }
                        if(runtime.seconds() - firstTime > 0.2){
                            if(done){
                                firstCheck = false;
                                firstTime = 0;
                                //stops all drive motors to ensure bot does not move location
                                drive(0,0,0);
                            }
                            done = moveToLocation(targetX, targetY, targetTheta, powerShotDistanceTolerance, powerShotRotationTolerance);
                        }
                        //moves to power shot 3*/
                        done = moveToLocation(targetX, targetY, targetTheta, powerShotDistanceTolerance, powerShotRotationTolerance);
                        break;

                    case "WAIT":
                        //wait step that can be added in if necessary for a debug of the code
                        if(runtime.seconds() > 0 && runtime.seconds() < 2) {
                            done = false;
                        }
                        else {
                            done = true;
                            changeStep();
                        }
                        break;

                    case "READ_ARM_POSITION":
                        //reads arm encoder position for future steps
                        armStartPosition = mA.getCurrentPosition();
                        done = true;
                        break;

                    case "SET_FOR_TELE":
                        //sets arm and elevator for the start of tele-op
                        goToAngle(0);  //moves elevator
                        sLower.setPosition(1);

                        targetX = 0;
                        targetY = 72;
                        targetTheta = 0;

                        if(frontLimit.isPressed()) {  //uses limit switch to move arm to a known position
                            limitTriggered = true;
                            armStartPosition = mA.getCurrentPosition();
                        }

                        else if (!limitTriggered) {
                            mA.setPower(-1);
                        }

                        sW.setPosition(1); //sets position of wrist
                        sG.setPosition(0.1);

                        moveToLocation(targetX, targetY, targetTheta, endDistanceTolerance, endRotationTolerance);

                        if (limitTriggered) {
                            armMotion = 3500;
                            armStopPosition = armStartPosition + armMotion; //calculates end arm position


                            if (mA.getCurrentPosition() >= armStopPosition) {

                                mA.setPower(0);
                                done = (moveToLocation(targetX, targetY, targetTheta, endDistanceTolerance, endRotationTolerance));
                                //done = true;
                                /*if(done){
                                    changeStep();
                                }*/
                            } else {
                                mA.setPower(1);
                            }


                        }
                        break;

                    case "GO_TO_LAUNCH_LINE":
                        //goes to launch line at the very end of autonomous
                        targetX = 0;
                        targetY = 72;
                        targetTheta = 0;
                        done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
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

    public void changeTargetZone (String zoneName){
        //sets coordinates for all three target zones, target zone is chosen depending on number of rings counted
        switch (zoneName) {

            case "A":
                targetZoneX = 26;
                targetZoneY = 68;
                break;

            case "B":
                targetZoneX = 2;
                targetZoneY = 88;
                break;

            case "C":
                targetZoneX = 26;
                targetZoneY = 112;
                break;

            default:
                targetZoneX = 0;
                targetZoneY = 0;
                break;

        }
    }
}





