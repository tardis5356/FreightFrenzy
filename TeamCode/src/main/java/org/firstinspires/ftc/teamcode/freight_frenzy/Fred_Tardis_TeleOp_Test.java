package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Fred_Tardis_TeleOp_Test", group = "Linear Opmode")
@Disabled
public class Fred_Tardis_TeleOp_Test extends BaseClass_FF {    // LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        defineComponentsFred();
        double powerMultiplier = 0.6;
        boolean previousBState = false;
        boolean previousRightBumperState = false;
        boolean previousX2State = false;
        boolean grippingCapstone = false;
        boolean motorPowerFast = false;
        boolean intaking = false;
        boolean wirelessConnected = true;
        double sVPosition = sV.getPosition();
//        double sCUPosition = sCU.getPosition();
        //used later to determine intake and outtake points for the arm
        double armLevelReading = mU.getCurrentPosition();
        double extensionPosition = mE.getCurrentPosition();
        armLimitOffset = 0;

        //01-02-21
        double intakeUpright = 2.7;
        double outtakeUpright = 0.53;
        double intake2Upright = 0.05;
        double neutralUpright = 0.95;

        double intakeWrist = 0.50;
        double outtakeWrist = 0.81;
        double intake2Wrist = 0.81;
        double neutralWrist = 0.81;

        double intakeExtension = 400;
        double outtakeExtension = 600;
        double intake2Extension = 800;
        double neutralExtension = 300;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updatePoseStrafe();
            gyroUpdate();

            //Gamepad 1 Variables
            // waitForStart();
            runtime.reset();

            double leftY1 = gamepad1.left_stick_y * powerMultiplier;//drive forward
            double rightX1 = -(gamepad1.right_stick_x) * powerMultiplier;//drive rotate
            double leftX1 = -(gamepad1.left_stick_x) * powerMultiplier;//drive strafe
            double rightY1 = -(gamepad1.right_stick_y);//free
            double rightTrigger = (gamepad1.right_trigger);//capstone arm
            double leftTrigger = (gamepad1.left_trigger);//capstone arm
            boolean rightBumper = (gamepad1.right_bumper);//capstone gripper
            boolean xButton = (gamepad1.x);//reverse spinner direction
            boolean bButton = (gamepad1.b);//turbo drive
            boolean yButton = (gamepad1.y);//intake reverse
            boolean aButton = (gamepad1.a);//spinner

            //Gamepad 2 Variables
            double rightTrigger2 = (gamepad2.right_trigger);//wrist up
            double rightY2 = (gamepad2.right_stick_y);//arm angle
            double leftY2 = (gamepad2.left_stick_y);//arm extend
            double leftTrigger2 = (gamepad2.left_trigger);//wrist down
            boolean leftBumper2 = (gamepad2.left_bumper);//free
            boolean rightBumper2 = (gamepad2.right_bumper);//free
            boolean bButton2 = (gamepad2.b);//sets arm for intake
            boolean aButton2 = (gamepad2.a);//sets arm for outtake
            boolean yButton2 = (gamepad2.y);//sets wrist for intake
            boolean xButton2 = (gamepad2.x);//sets wrist for outtake

            telemetry.addData("leftTrigger2", leftTrigger2);
            telemetry.addData("rightTrigger2", rightTrigger2);
            telemetry.addData("intake servo power", sI.getPower());
            telemetry.update();

            double intakePower = 1;

            //controls intake
            //trigger pressed = 1, trigger not pressed = 0
            if (leftTrigger2 >= 0.5) {
                //sucks elements in
                sI.setPower(intakePower);
            } else if (rightTrigger2 >= 0.5) {

                sI.setPower(-intakePower);
            } else {

                sI.setPower(0);
            }

            previousRightBumperState = rightBumper;
            previousBState = bButton;
            previousX2State = xButton2;
            extensionPosition = mE.getCurrentPosition();
        }


    }


}
