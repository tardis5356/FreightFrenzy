
package org.firstinspires.ftc.teamcode.freight_frenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appea
 *
 * r on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DemoRobot_TeleOp", group="Linear Opmode")

public class DemoRobot_TeleOp extends BaseClass_FF {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    double wristPosition = 0.5;

    @Override
    public void runOpMode() {
        defineComponentsDemoRobot();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        mL = hardwareMap.get(DcMotor.class, "mL");
        mA = hardwareMap.get(DcMotor.class, "mA");
        mR = hardwareMap.get(DcMotor.class, "mR");

        sG = hardwareMap.servo.get("sG");
        sW = hardwareMap.servo.get("sW");

        mL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        mL.setDirection(DcMotor.Direction.FORWARD);
        mR.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Gamepad 1 Variables
            double leftY1 = gamepad1.left_stick_y;
            double rightX1 = (gamepad1.right_stick_x) / 2;


            //Gamepad 2 Variables
            double leftY2 = gamepad2.left_stick_y;
            double rightY2 = gamepad2.right_stick_y;
            double rightTrigger2 = gamepad2.right_trigger;
            double leftTrigger2 = gamepad2.left_trigger;
            boolean leftBumper2 = gamepad2.left_bumper;
            boolean rightBumper2 = gamepad2.right_bumper;


            mL.setPower(leftY1 - rightX1);
            mR.setPower(leftY1 + rightX1);

            if (rightTrigger2 != 0) {
                sG.setPosition(0.3);
            } else {
                sG.setPosition(0.85);
            }
            //mGA.setPower(leftY2);

            //controls wrist, moves in increments
            if (rightBumper2 && wristPosition < 1) {
                wristPosition += .006;
            } else if (leftBumper2 && wristPosition > 0) {
                wristPosition -= .006;
            }
            sW.setPosition(Range.clip(wristPosition, 0.1, 0.83));

            mA.setPower(-rightY2);

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            //mL.setPower(leftPower);
            //mR.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("wrist position", "" + String.format("%.2f", wristPosition));
            telemetry.update();
        }
    }
}





/*package org.firstinspires.ftc.teamcode.skystone.TB1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "TeleOp_DemoRobot", group = "TeleOp")


public abstract class TeleOp_DemoRobot extends BaseClass_DemoRobot {

    @Override
    public void runOpMode() {

        defineComponents();

        waitForStart();

        while (opModeIsActive()) {
            double leftX1 = gamepad1.left_stick_x;
            double leftY1 = gamepad1.left_stick_y;
            double rightY1 = gamepad1.right_stick_y;
            double rightTrigger1 = gamepad1.right_trigger;
            double leftTrigger1 = gamepad1.left_trigger;

            if (rightTrigger1 != 0) {
                sG.setPosition(1);
            } else {
                sG.setPosition(0);
            }

            if (leftTrigger1 != 0) {
                sGA.setPosition(1);
            } else {
                sGA.setPosition(0);
            }

            mA.setPower(rightY1);
        }
    }
}*/

