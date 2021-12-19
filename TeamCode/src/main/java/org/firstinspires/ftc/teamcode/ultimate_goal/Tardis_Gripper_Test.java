package org.firstinspires.ftc.teamcode.ultimate_goal;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tardis_Gripper_Test", group = "Linear Opmode")
@Disabled

public class Tardis_Gripper_Test extends BaseClassTB1 {

    @Override
    public void runOpMode() {

        defineComponents();

        waitForStart();

        while (opModeIsActive()) {
            double rightTrigger2 = (gamepad2.right_trigger);

            if (rightTrigger2 != 0){
                sG.setPosition(0.5); //sets gripper to closed
            } else {
                sG.setPosition(0.8); //sets gripper to open
            }
        }
    }
}
