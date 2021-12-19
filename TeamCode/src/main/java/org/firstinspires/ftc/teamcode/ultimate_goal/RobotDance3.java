package org.firstinspires.ftc.teamcode.ultimate_goal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.ArrayList;

@Autonomous(name = "RobotDance3", group = "Autonomous")
@Disabled

public class RobotDance3 extends AutoBaseTB1 {

    ArrayList<String> danceSteps = new ArrayList<>();

    public void CreateDanceSteps() {
        //danceSteps.add("MOVE_TO_NW_CORNER");
        //danceSteps.add("MOVE_TO_NE_CORNER");
        //danceSteps.add("MOVE_TO_SE_CORNER");
        //danceSteps.add("MOVE_TO_SW_CORNER");
        danceSteps.add("ROTATE_TO_NE");
        danceSteps.add("ROTATE_TO_NW");
        danceSteps.add("ROTATE_TO_SW");
        danceSteps.add("ROTATE_TO_SE");
        danceSteps.add("MOVE_TO_HOME");
   //     danceSteps.add("FULL_TURN_PART_1");
     //   danceSteps.add("FULL_TURN_PART_2");
        // danceSteps.add("MOVE_TO_HOME");
       // danceSteps.add("DO_A_180");
        danceSteps.add("STOP");

    }



    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        boolean done = false;
        CreateDanceSteps();

        //Init functions
        defineComponents();

        double targetX = 0;
        double targetY = 0;
        double targetTheta = 0;
        double distanceTolerance = 2;
        double rotationTolerance = 2;

        waitForStart();

        //Reset time variables
        runtime.reset();

        // loops through all steps the robot is supposed to perform
            for (String currentStep : danceSteps) {
                done = false;      // reset so we can run the next step...

                // decides which step we are on, and runs that action
                while (opModeIsActive() && (done == false)) {
                    telemetry.addData("gyro", "" + String.format("%.2f deg", gyroZ));
                    telemetry.addData("current position x", "" + String.format("%.2f in.", pose.x));
                    telemetry.addData("current position y", "" + String.format("%.2f in.", pose.y));
                    telemetry.addData("current step", currentStep);
                    telemetry.update();
                    //Update global sensor values
                    updatePoseStrafe();
                    gyroUpdate();

                    switch (currentStep) {

                        case "FULL_TURN_PART_1":
                            targetX = 0;
                            targetY = 0;
                            targetTheta = 135;
                            done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                            break;

                        case "FULL_TURN_PART_2":
                            targetX = 0;
                            targetY = 0;
                            targetTheta = 0;
                            done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                            break;

                        case "DO_A_180":
                            targetX = 0;
                            targetY = 0;
                            targetTheta = 180;
                            done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                            break;

                        case "DO_A_90":
                            targetX = 0;
                            targetY = 0;
                            targetTheta = 90;
                            done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                            break;

                        case "COME_FULL_CIRCLE":
                            targetX = 0;
                            targetY = 0;
                            targetTheta = 360;
                            done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                            break;

                        case "MOVE_TO_HOME":
                            targetX = 0;
                            targetY = 0;
                            targetTheta = 0;
                            done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                            break;

                        case "ROTATE_TO_NE":
                            targetX = 24;
                            targetY = 24;
                            targetTheta = -45;
                            done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                            break;

                        case "ROTATE_TO_NW":
                            targetX = -24;
                            targetY = 24;
                            targetTheta = 45;
                            done = moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance);
                            break;

                        case "ROTATE_TO_SW":
                            targetX = -24;
                            targetY = -24;
                            targetTheta = 135;
                            done = moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance);
                            break;

                        case "ROTATE_TO_SE":
                            targetX = 24;
                            targetY = -24;
                            targetTheta = 225;
                            done = moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance);
                            break;

                        case "STOP":
                            drive(0, 0, 0);
                            done = true;
                            break;

                    }
                }
            }
        shutdown();
    }


    public boolean moveToLocation
            (double targetX, double targetY, double targetTheta, double distanceTolerance, double rotationTolerance) {
        if (isInTolerance(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance)) {
            changeStep(); // you are within tolerance, so stop the drive train and move to next step
            return true;
        } else {
            moveToPose(targetX, targetY, targetTheta, 50);
            return false;
        }

    }



}




