//package org.firstinspires.ftc.teamcode.freight_frenzy;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//@TeleOp(name = "Primus_TeleOp", group = "Linear Opmode")
//
//public class Primus_TeleOp extends BaseClass_FF {    // LinearOpMode {
//
//    private ElapsedTime runtime = new ElapsedTime();
//
//    @Override
//    public void runOpMode() {
//
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
//
//        defineComponentsPrimus();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            //Gamepad 1 Variables
//            double leftY1 = gamepad1.left_stick_y;
//            double rightX1 = gamepad1.right_stick_x;
//
//            //Gamepad 2 Variables
//            double leftY2 = gamepad2.left_stick_y;
//            double rightTrigger2 = gamepad2.right_trigger;
//            double leftTrigger2 = gamepad2.left_trigger;
//
//            mBL.setPower(leftY1 - rightX1);
//            mBR.setPower(leftY1 + rightX1);
//            mFL.setPower(leftY1 - rightX1);
//            mFR.setPower(leftY1 + rightX1);
//
//            sArm.setPower(leftY2);
//
//            if(rightTrigger2 != 0){
//                sL.setPosition(0.75);
//                sR.setPosition(0.75);
//            } else if(leftTrigger2 != 0){
//                sL.setPosition(0.2);
//                sR.setPosition(0.2);
//            } else {
//                sL.setPosition(0);
//                sR.setPosition(0);
//            }
//
//        }
//
//
//    }
//
//
//}
