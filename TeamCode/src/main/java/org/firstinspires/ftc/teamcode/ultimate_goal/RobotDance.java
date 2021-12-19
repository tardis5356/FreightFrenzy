package org.firstinspires.ftc.teamcode.ultimate_goal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "RobotDance", group = "Autonomous")
@Disabled

public class RobotDance extends AutoBaseTB1 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        //Initializes first step
     //   steps CURRENT_STEP = steps.DELIVER_WOBBLE;
        steps CURRENT_STEP = steps.MOVE_TO_NE_CORNER;


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

       while (opModeIsActive()) {
            telemetry.addData("gyro", gyroZ);
            telemetry.addData("current position x", pose.x);
            telemetry.addData("current position y", pose.y);
            telemetry.update();
            //Update global sensor values
            updatePoseStrafe();
            gyroUpdate();

            switch (CURRENT_STEP) {

                case MOVE_TO_NE_CORNER:
                    targetX = 24;
                    targetY = 24;
                    targetTheta = 0;

                    if(isInTolerance(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance)) {
                       changeStep();
                        CURRENT_STEP = steps.MOVE_TO_NW_CORNER;
                    }
                        else {
                        // Go to NE corner
                        moveToPose(targetX, targetY, targetTheta, 50);
                    }
                    break;

               case MOVE_TO_NW_CORNER:
                    targetX = -24;
                    targetY = 24;
                    targetTheta = 0;

                    if(isInTolerance(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance)) {
                        changeStep();
                        CURRENT_STEP = steps.MOVE_TO_SW_CORNER;
                    }
                    else {
                        // Go to NW corner
                        moveToPose(targetX, targetY, targetTheta, 50);
                    }
                    break;

                case MOVE_TO_SW_CORNER:
                    targetX = -24;
                    targetY = -24;
                    targetTheta = 0;

                    if(isInTolerance(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance)) {
                        changeStep();
                        CURRENT_STEP = steps.MOVE_TO_SE_CORNER;
                    }
                    else {
                        // Go to SW corner
                        moveToPose(targetX, targetY, targetTheta, 50);
                    }
                    break;

                case MOVE_TO_SE_CORNER:
                    targetX = 24;
                    targetY = -24;
                    targetTheta = 0;

                    if(isInTolerance(targetX, targetY, targetTheta, 2, rotationTolerance)) {
                        changeStep();
                        CURRENT_STEP = steps.GO_BACK_HOME;
                      //  CURRENT_STEP = steps.GO_BACK_HOME_SPIN;
                    }
                    else {
                        // Go to SE corner
                        moveToPose(targetX, targetY, targetTheta, 50);
                    }
                    break;


                case GO_BACK_HOME:
                    targetX = 0;
                    targetY = 0;
                    targetTheta = 0;

                    if(isInTolerance(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance)) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    }
                    else {
                        // Go back home
                        moveToPose(targetX, targetY, targetTheta, 50);
                    }
                    break;

                case GO_BACK_HOME_SPIN:
                    targetX = 0;
                    targetY = 0;
                    targetTheta = 90;

                    if(isInTolerance(targetX, targetY, targetTheta, distanceTolerance, rotationTolerance)) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    }
                    else {
                        // Go back home
                        moveToPose(targetX, targetY, targetTheta, 50);
                    }
                    break;

                case STOP:
                    drive(0, 0, 0);
                    break;

            }
        }
        shutdown();
    }
}