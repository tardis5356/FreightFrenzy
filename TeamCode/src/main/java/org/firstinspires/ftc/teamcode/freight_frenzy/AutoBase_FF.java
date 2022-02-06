package org.firstinspires.ftc.teamcode.freight_frenzy;

//Imports


public abstract class AutoBase_FF extends BaseClass_FF {

    //Check for push and pull
    //Global variables
    double timeAtStop = 0;
    String hubLevel = "TOP";
    double armAngle = 0;
    double armReach = 0;
    double wristPosition = 0;
    double telescopePose = 0;
    //armHorizontal = 1.86;
    //wristStraight = 0;




    public void fredTelemetry() {

        telemetry.addData("Hub level", hubLevel);
        telemetry.addData("gyro", "" + String.format("%.2f deg", gyroZ));
        telemetry.addData("back distance (in)", "" + String.format("%.2f", backDistance));
        telemetry.addData("back distance filtered (in)", "" + String.format("%.2f", backDistanceFiltered));
        telemetry.addData("front distance (in)", "" + String.format("%.2f", frontDistance));
        telemetry.addData("front distance filtered (in)", "" + String.format("%.2f", frontDistanceFiltered));
        telemetry.addData("left distance (in)", "" + String.format("%.2f", leftDistance));
        telemetry.addData("left distance filtered (in)", "" + String.format("%.2f", leftDistanceFiltered));
        telemetry.addData("right distance (in)", "" + String.format("%.2f", rightDistance));
        telemetry.addData("right distance filtered (in)", "" + String.format("%.2f", rightDistanceFiltered));
        telemetry.addData("potentiometer angle", potentiometer.getVoltage());
        telemetry.addData("extension position", mE.getCurrentPosition());
        telemetry.addData("arm limit", lAB.isPressed());
//        telemetry.addData("arm limit offset", armLimitOffset);
        telemetry.addData("Wrist angle", sV.getPosition());
        telemetry.update();
    }

    public void gingerTelemetry() {

        telemetry.addData("gyro", "" + String.format("%.2f deg", gyroZ));
        telemetry.addData("front distance (in)", "" + String.format("%.2f", frontDistance));
        telemetry.addData("front distance filtered (in)", "" + String.format("%.2f", frontDistanceFiltered));
        telemetry.addData("left distance (in)", "" + String.format("%.2f", leftDistance));
        telemetry.addData("left distance filtered (in)", "" + String.format("%.2f", leftDistanceFiltered));
        telemetry.addData("sX position", sX.getPosition());
        telemetry.addData("sYL position", sYL.getPosition());
        telemetry.addData("sYR position", sYR.getPosition());
        telemetry.addData("x pos",pose.x);
        telemetry.addData("y pos",pose.y);
        telemetry.update();
    }


//    public void resetArm() {
//
//        drive(0,0,0);
//        double armAngleBack = armHorizontal;
////        telemetry.addData("target arm angle", armAngle);
////        telemetry.addData("target arm extension", armReach);
////        telemetry.addData("arm extension", mE.getCurrentPosition());
////        telemetry.addData("telescope pose (offset)", telescopePose);
//        sV.setPosition(0.1);
//        if(lAB.isPressed()) {  //uses limit switch to move arm to a known position
//            telescopePose = mE.getCurrentPosition();
//            mE.setPower(0);
//            extendDone = true;
//        }
//
//        else if (!lAB.isPressed()) {
//            mE.setPower(-1);
//        }
//
//        if ((Math.abs(potentiometer.getVoltage() - armAngleBack) > potTolerance) && extendDone) {
//            if (potentiometer.getVoltage() > armAngleBack) {
//
//                mU.setPower(-0.5);
//            } else if (potentiometer.getVoltage() < armAngleBack) {
//
//                mU.setPower(0.5);
//            }
//        } else {
//
//            mU.setPower(0);
//            angleDone = true;
//
//        }
////        telemetry.addData("arm done", angleDone);
//
//    }

    public void changeHubLevel(String hubLevel) {
        //sets coordinates for all three target zones, target zone is chosen depending on position of team scoring element
        switch (hubLevel) {
            //43 extension ticks per cm

            case "BOTTOM":

                armAngle = armHorizontal + 1.11;
                //3.0
                armReach = telescopePose + 409;
                wristPosition = 0.41 + wristStraight;
                //0.8
                break;

            case "MIDDLE":

//                armAngle = 2.5;
                //test values for extension testing
                armAngle = armHorizontal; //+ 0.5;
                armReach = telescopePose + 1029; //+ 237;
                //364
                wristPosition = 0.48 + wristStraight;
                break;

            case "TOP":

                //1.65
                //1.55
                armAngle = armHorizontal - 0.35;
                //0.3
                wristPosition = wristStraight;
                //700
                armReach = telescopePose + 895;
                break;

            default:

                armAngle = armHorizontal - 0.35;
                wristPosition = 0.21 + wristStraight;
                armReach = telescopePose + 895;
                break;

        }
    }

    //Drives forward while correcting to face designated gyro heading
    public void driveForwardGyro(double power, double degree, double tolerance) {
        if (gyroZ + tolerance < degree) {
            mFL.setPower(power);
            mFR.setPower(power + 0.3);
            mBL.setPower(power);
            mBR.setPower(power + 0.3);
        } else if (gyroZ - tolerance > degree) {
            mFL.setPower(power + 0.3);
            mFR.setPower(power);
            mBL.setPower(power + 0.3);
            mBR.setPower(power);
        } else {
            driveForward(power);
        }
    }

    //Drives right while correcting to face designated gyro heading
    public void driveRightGyro(double power, double degree, double tolerance) {
        if (gyroZ + tolerance < degree) {
            mFL.setPower(power);
            mFR.setPower(-power + 0.1);
            mBL.setPower(-power);
            mBR.setPower(power + 0.1);
        } else if (gyroZ - tolerance > degree) {
            mFL.setPower(power + 0.1);
            mFR.setPower(-power);
            mBL.setPower(-power + 0.1);
            mBR.setPower(power);
        } else {
            driveRight(power);
        }
    }

    public void gyroAdjust(double power, double degree) {

        if (gyroZ > degree) {
            rotateClockwise(power);
        } else if (gyroZ < degree) {
            rotateCounterclockwise(power);
        } else {
            return;
        }
    }



    public void exactPosition(double degree, double degTol, double posX, double posY) {
        if (gyroZ + degTol < degree || gyroZ - degTol > degree) {
            gyroAdjust(degree, degTol);
        } else {
            if (Math.abs(posX - rightDistance) >= Math.abs(posY - backDistance)) {
                if (backDistance <= posY) {
                    drive(0.2,0,0);
                } else {
                    drive(0.2,0,0);
                }
            } else {
                if (rightDistance <= posX) {
                    drive(0,0,0.3);
                } else {
                    drive(0,0,0.3);
                }
            }
        }
    }

    public void preciseRotationChange(double targetAngle, double totalAngleChange) {
        double rotationAggressiveness = 1/ Math.abs(totalAngleChange);
        double powerThreshold = 0.5;
        //smaller maximum powers make turn slower, but slightly more accurate
        double pMax = 1;
        double power = 2 * pMax * ((1 / (1 + Math.pow(Math.E, -(rotationAggressiveness * (gyroZ - targetAngle))))) - 0.5);
        if (power < powerThreshold && power > 0) {
            power = powerThreshold;
        }
        if (power > -powerThreshold && power < 0) {
            power = -powerThreshold;
        }
        //switched from rotateClockwise to rotateCounterclockwise to accommodate turning on testbed - 8/22/21
        rotateClockwise(power);
        //rotateCounterclockwise(power);

    }

    //Sigmoid function for rotating clockwise
    public void rotateSigmoid(double degree) {
        double rotationAggressiveness = 0.08;
        //0.05
        double powerThreshold = 0.15;
        double pMax = 0.5;
        //0.5
        double power = 2 * pMax * ((1 / (1 + Math.pow(Math.E, -(rotationAggressiveness * (gyroZ - degree))))) - 0.5);
        if (power < powerThreshold && power > 0) {
            power = powerThreshold;
        }
        if (power > -powerThreshold && power < 0) {
            power = -powerThreshold;
        }
        //switched from rotateClockwise to rotateCounterclockwise to accommodate turning on testbed - 8/22/21s
        //rotateClockwise(power);
        rotateCounterclockwise(power);
    }

    //Displays all sensor readings and important variables for debugging
    public void sensorTelemetry() {
        gyroUpdate();

        telemetry.addData("Gyro: ", gyroZ);
        telemetry.addData("Runtime: ", String.format("%.01f sec", runtime.seconds()));
        telemetry.addData("mBR ticks", mBR.getCurrentPosition());
        telemetry.update();
    }

    public void moveToPoseUpdates(double targetX, double targetY, double gyroLock, boolean drive) {

        double distanceX = (targetX - pose.x);
        double distanceY = (targetY - pose.y);
        double distanceTheta = gyroLock - gyroZ;

        double leftX = distanceX / 20;
        double leftY = -distanceY / 20;
        double rightX = distanceTheta / 180;

        double strafeAssist = 0;

        /*
        if(gyroZ > gyroLock + 2) {
            strafeAssist = (gyroZ - gyroLock) / 25;
        } else if(gyroZ < gyroLock - 2) {
            strafeAssist = (gyroZ - gyroLock) / 25;
        } else {
            strafeAssist = 0;
        }
        */

        double mFLPower = (leftX + leftY) * Math.sin(Math.toRadians(gyroZ) + (Math.PI / 4)) - rightX + strafeAssist;
        double mFRPower = (leftX - leftY) * Math.cos(Math.toRadians(gyroZ) + (Math.PI / 4)) + rightX - strafeAssist;
        double mBLPower = (leftX - leftY) * Math.cos(Math.toRadians(gyroZ) + (Math.PI / 4)) - rightX + strafeAssist;
        double mBRPower = (leftX + leftY) * Math.sin(Math.toRadians(gyroZ) + (Math.PI / 4)) + rightX - strafeAssist;

        telemetry.addData("mFL", mFLPower);
        telemetry.addData("mFR", mFRPower);
        telemetry.addData("mBL", mBLPower);
        telemetry.addData("mBR", mBRPower);
        telemetry.update();

        if (drive) {
            mFL.setPower(mFLPower);
            mFR.setPower(mFRPower);
            mBL.setPower(mBLPower);
            mBR.setPower(mBRPower);
        }
    }


    public void rotateToPose(double targetX, double targetY) {
        double distanceX = (targetX - pose.x);
        double distanceY = (targetY - pose.y);
        double tangentOf = (targetY - pose.y) / distanceX;

        double distanceToTheta = (Math.atan(tangentOf) - pose.theta);

        rotateSigmoid(Math.toDegrees(Math.atan(tangentOf)));

        telemetry.addData("Gyro", gyroZ);
        telemetry.addData("Target", Math.toDegrees(Math.atan(tangentOf)));
        telemetry.update();

    }

    //Shutdown all processes and stop drivetrain
    public void shutdown() {
        stopDriveTrain();
        stop();
    }

    public void changeStep() {
        runtime.reset();
        timeAtStop = stopTime.seconds();
        stopDriveTrain();
        isStartRecorded = false;
    }

    //Cases for auto steps
    public enum steps {
      /*  MOVE_TO_NE_CORNER,
        MOVE_TO_NW_CORNER,
        MOVE_TO_SW_CORNER,
        MOVE_TO_SE_CORNER,
        MOVE_TO_HOME,
        STOP
*/

        MOVE_TO_EAST,
        MOVE_TO_NE_CORNER,
       // MOVE_TO_EAST,
       // MOVE_TO_NE_CORNER,
       // PREP_TO_SHOOT,
        DELIVER_WOBBLE,
        GO_BACK_HOME,
        GO_BACK_HOME_SPIN,
        MOVE_TO_NW_CORNER,
        MOVE_TO_SW_CORNER,
        MOVE_TO_SE_CORNER,
        STOP



    }
}