package org.firstinspires.ftc.teamcode.ultimate_goal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.ArrayList;

@Autonomous(name = "RobotDanceNEW", group = "Autonomous")
@Disabled

public class RobotDanceNEW extends AutoBaseTB1 {


    //Cases for auto steps
    public enum danceSteps {
        MOVE_TO_NW_CORNER("1st"),
        MOVE_TO_SW_CORNER("1st"),
        MOVE_TO_NE_CORNER("1st"),
        MOVE_TO_SE_CORNER("1st"),
       // MOVE_TO_SE_CORNER,
        MOVE_TO_HOME("1st"),
        STOP("1st");

        danceSteps(String stepID) {
        }
    }


    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        boolean done = false;

        //Init functions
        defineComponents();

        double targetX = 0;
        double targetY = 0;
        double targetTheta = 0;
        double distanceTolerance = 5;
        double rotationTolerance = 5;

        waitForStart();

        //Reset time variables
        runtime.reset();

        // loops through all steps the robot is supposed to perform
        for (int i = 0; i <= 2; i++) {
            for (danceSteps currentStep : danceSteps.values()) {
                done = false;      // reset so we can run the next step...

                // decides which step we are on, and runs that action
                while (opModeIsActive() && (done == false)) {
                    //telemetry.addData("gyro", gyroZ);
                   // telemetry.addData("current position x", pose.x);
                    telemetry.addData("gyro", "" + String.format("%.2f deg", gyroZ));
                    telemetry.addData("current position x", "" + String.format("%.2f in.", pose.x));
                    telemetry.addData("current position y", "" + String.format("%.2f in.", pose.y));
                   // telemetry.addData("current position y", pose.y);
                    telemetry.addData("current step", currentStep.toString());
                    //telemetry.addData("step status = ", String.valueOf(done));
                    telemetry.update();
                    //Update global sensor values
                    updatePoseStrafe();
                    gyroUpdate();

                    switch (currentStep) {

                        case MOVE_TO_HOME:
                            targetX = 0;
                            targetY = 0;
                            targetTheta = 0;
                            done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                            break;

                        case MOVE_TO_NE_CORNER:
                            targetX = 24;
                            targetY = 24;
                            targetTheta = 0;
                            done = (moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance));
                            break;

                        case MOVE_TO_NW_CORNER:
                            targetX = -24;
                            targetY = 24;
                            targetTheta = 0;
                            done = moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance);
                            break;

                        case MOVE_TO_SW_CORNER:
                            targetX = -24;
                            targetY = -24;
                            targetTheta = 0;
                            done = moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance);
                            break;

                        case MOVE_TO_SE_CORNER:
                            targetX = 24;
                            targetY = -24;
                            targetTheta = 0;
                            done = moveToLocation(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance);
                            break;

                        case STOP:
                            drive(0, 0, 0);
                            done = true;
                            break;

                    }
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




