package org.firstinspires.ftc.teamcode.ultimate_goal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SuperAutoDestruct {

    @TeleOp(name = "AutoDestruct", group = "Linear Opmode")
    @Disabled

    public class AutoDestruct extends BaseClassTB1 {    // LinearOpMode {

        private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor mFL = null;
//   private DcMotor mFR = null;
// changed motor names in entire program to mFL and mFR from mL and mR

        @Override
        public void runOpMode() {

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            double autoDestructVariable = 0.987654321;

            while (opModeIsActive()) {
                telemetry.addData("You fool! Auto destruct initialized. Robot core meltdown has begun. *maniacal laughter* ", autoDestructVariable);
            }
        }
    }
}