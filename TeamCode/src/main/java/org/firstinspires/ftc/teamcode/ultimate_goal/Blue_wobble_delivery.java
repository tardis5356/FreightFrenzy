package org.firstinspires.ftc.teamcode.ultimate_goal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name = "Blue_wobble_delivery", group = "Autonomous")
@Disabled

public class Blue_wobble_delivery extends AutoBaseTB1 {

    @Override
    public void runOpMode() {//Start of the initiation for autonomous

        //Initializes first step
      //  steps CURRENT_STEP = steps.DELIVER_WOBBLE;
        steps CURRENT_STEP = steps.MOVE_TO_EAST;

        //Init functions
        defineComponents();

        double targetX = 0;
        double targetY = 0;

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

                case DELIVER_WOBBLE:
                    targetX = 0;
                    targetY =36;
                    double targetTheta = 45;

                    if(isInTolerance(targetX, targetY, targetTheta, 4, 180)) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                    }

                    moveToPose(targetX, targetY, targetTheta, 1);
                    break;

                case STOP:
                    drive(0, 0, 0);
                    break;

            }

        }
        shutdown();
    }


}

