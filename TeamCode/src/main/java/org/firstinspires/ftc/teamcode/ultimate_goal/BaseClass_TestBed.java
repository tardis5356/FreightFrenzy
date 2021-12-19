

package org.firstinspires.ftc.teamcode.ultimate_goal;

//import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ultimate_goal.Pose;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.util.Range;


public abstract class BaseClass_TestBed extends LinearOpMode {

    //Final variables (in inches)
    final static double wheelDiameter = 2.375;
    final static double wheelDistance = 16.25;

    OpenCvCamera webcam;
    CountRings.SkystoneDeterminationPipeline pipeline;


    DcMotor mBL;//Back left
    DcMotor mBR;
    DcMotor mFL;
    DcMotor mFR;//Front right
    DcMotor mA; //Arm
    DcMotor mI; //Intake
   // DcMotor mI; //Intake
    //DcMotor mE; //Elevator
   // DcMotor mS; //Shooter/spinner
    //AnalogInput potentiometer;
    Servo sW; //Wrist
    Servo sG; //Gripper
    //Servo sLower; //intake lower
    //Servo sP; //pusher
    //TouchSensor frontLimit; //front limit
    //TouchSensor backLimit; //back limit
    //BNO055IMU imuControl; //REV gyro - Control hub
    //BNO055IMU imuExpansion;   // expansion hub gyro
    //Orientation angles;
    ElapsedTime runtime;
    ElapsedTime runtimeTwo;
    ElapsedTime stoneIntakeTime;
    //ColorSensor c1;
    DistanceSensor distance1;
    DistanceSensor distance2;
    ModernRoboticsI2cRangeSensor rangeSensorLeft;
    ModernRoboticsI2cRangeSensor rangeSensorRight;
    ElapsedTime stopTime;
    //Orientation lastAngle = new Orientation();
    Pose pose = new Pose(0, 0, 0);
    //Global sensor values
    float gyroZ = 0;        // iniializing at zero just to be sure
    boolean isStartRecorded = false;
    //moveToPose() global
    double thetaStart;
    double distanceToTargetStart;
    double thetaRobot;
    //Global variables
    int currOdY1;
    int currOdY2;
    int prevOdY1 = 0;
    int prevOdY2 = 0;
    int deltaOdY1 = 0;
    int deltaOdY2 = 0;
    double currTheta = 0;
    double deltaTheta = 0;
    double arcLength = 0;
    int prevEnX = 0;
    int prevEnYLeft = 0;
    //double enXprediction = 0;
    int prevEnYRight = 0;
    double prevGyro = 0;
    double posX = 0;
    double posY = 0;
    double targetElevAngle = 0;
    int encoderXStart;
    int encoderYLeftStart;
    int encoderYRightStart;
    double currFDist;
    double currBDist;
    double currRDistRange;
    double currLDistRange;

//    public double getElevAngle(double voltage) {
//
//        double elevAngle;
//
//        //double targetPotVolt = 0.0103 * targetElevAngle + 1.074;
//        //double targetPotVolt = 0.01057 * targetElevAngle + 1.0879;
//        double targetPotVolt = 0.009864 * targetElevAngle + 1.10809; // 5/16
//        //elevAngle = (voltage - 1.074)/0.0103; OLD
//        //elevAngle = (voltage - 1.0879)/0.01057;
//        elevAngle = (voltage - 1.10809)/0.009864; // 5/16
//        return elevAngle;
//    }
//    public void goToAngle(double inputElevAngle) {
//        //angle units are in degrees
//        targetElevAngle = Range.clip(inputElevAngle, -5, 35);
//        //double targetPotVolt = 0.01057 * targetElevAngle + 1.0879;
//        double targetPotVolt = 0.009864 * targetElevAngle + 1.10809; // 5/16
//        double voltage = potentiometer.getVoltage();
//        double voltageThreshold = 0.01;
//        if (voltage - targetPotVolt > voltageThreshold) {
//
//            mE.setPower(0.8);
//        } else if (voltage - targetPotVolt < -voltageThreshold) {
//
//            mE.setPower(-0.8);
//        } else {
//
//            mE.setPower(0);
//        }
//    }

    public static double ticksToInches(int ticks) {
        double circum = Math.PI * wheelDiameter;
        return (circum / 8000) * ticks;
    }

    public static double[] arcInfo(int deltaLeft, int deltaRight) {
        double inchesLeft = ticksToInches(deltaLeft);
        double inchesRight = ticksToInches(deltaRight);
        double deltaTheta = (inchesRight - inchesLeft) / wheelDistance;
        double arcLength = (inchesRight + inchesLeft) / 2;
        return new double[]{deltaTheta, arcLength};
    }

    //Defines all components for init()
    public void defineComponents() {

        mBL = hardwareMap.dcMotor.get("mBL");//Back left
        mBR = hardwareMap.dcMotor.get("mBR");
        mFL = hardwareMap.dcMotor.get("mFL");
        mFR = hardwareMap.dcMotor.get("mFR");//Front right
        mA = hardwareMap.dcMotor.get("mA");
        mI = hardwareMap.dcMotor.get("mI");
//        mI = hardwareMap.dcMotor.get("mI");
//        mE = hardwareMap.dcMotor.get("mE");
//        mS = hardwareMap.dcMotor.get("mS");
        sW = hardwareMap.servo.get("sW");
        sG = hardwareMap.servo.get("sG");
        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
        distance2 = hardwareMap.get(DistanceSensor.class, "distance2");
        rangeSensorLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_left");
        rangeSensorRight = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_right");

//        sP = hardwareMap.servo.get("sP");
//        sLower = hardwareMap.servo.get("sLower");
//        frontLimit = hardwareMap.get(TouchSensor.class, "frontLimit");
//        backLimit = hardwareMap.get(TouchSensor.class, "backLimit");
//        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; //Calibration file
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imuExpansion = hardwareMap.get(BNO055IMU.class, "imuExpansion");
        imuExpansion.initialize(parameters);*/


        //c1 = hardwareMap.get(ColorSensor.class, "c1");

        mBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set motors of robot backwards
        mBR.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.REVERSE);
//        mS.setDirection(DcMotor.Direction.REVERSE);
//        mI.setDirection(DcMotor.Direction.REVERSE);
//        mE.setDirection(DcMotor.Direction.REVERSE);


        //mFL.setDirection(DcMotor.Direction.REVERSE);
        mBL.setDirection(DcMotor.Direction.REVERSE);

        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mI.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // mE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime = new ElapsedTime();
        runtimeTwo = new ElapsedTime();
        stoneIntakeTime = new ElapsedTime();
        stopTime = new ElapsedTime();

        //0s out encoders
        encoderXStart = mFR.getCurrentPosition();
        encoderYLeftStart = mFL.getCurrentPosition();
        encoderYRightStart = mBL.getCurrentPosition();

    }

    public void squareOnWallRange() {
        currLDistRange = rangeSensorLeft.getDistance(DistanceUnit.CM);

        currRDistRange = rangeSensorRight.getDistance(DistanceUnit.CM);
        //if distance value is over 200, set it to 200 to eliminate crazy squaring
        if(currRDistRange > 200) {
            currRDistRange = 200;
        }

        if(currLDistRange > 200) {
            currLDistRange = 200;
        }
        //if value is less than 2 cm, don't square
        if(Math.abs(currLDistRange - currRDistRange) > 0.5) {
            drive(0, 0, -(currLDistRange - currRDistRange) / 10); //+
        }else{
            drive(0,0,0);
        }//else if (Math.abs(currLDistRange - currRDistRange) > 2){
         //   drive(0, 0, -(currLDistRange-currRDistRange)/10); //-
        }


    public void squareOnWall() {
        currFDist = distance1.getDistance(DistanceUnit.CM);
        currBDist = distance2.getDistance(DistanceUnit.CM);

        if(currFDist > currBDist){
            drive(0, 0, (currFDist-currBDist)/10); //+
        }else{
            drive(0, 0, -(currBDist-currFDist)/10); //-
        }
    }

    public Pose getPose() {
        return this.pose;
    }

    public void updatePose() {
        currOdY1 = mBL.getCurrentPosition();
        currOdY2 = mFR.getCurrentPosition();

        deltaOdY1 = currOdY1 - prevOdY1;
        deltaOdY2 = currOdY2 - prevOdY2;

        deltaTheta = arcInfo(deltaOdY1, deltaOdY2)[0];
        arcLength = arcInfo(deltaOdY1, deltaOdY2)[1];

        posX += arcLength * Math.cos(currTheta + (deltaTheta / 2));
        posY += arcLength * Math.sin(currTheta + (deltaTheta / 2));

        prevOdY1 = currOdY1;
        prevOdY2 = currOdY2;
        currTheta += deltaTheta;

        pose.x = posX;
        pose.y = posY;
        pose.theta = currTheta;
    }

    public void updatePoseStrafe() {

        //Set current values for iteration

        //all port numbers for expansion hub
        int currEnX = -mFR.getCurrentPosition() + encoderXStart;  // Motor Front Right & Odometer X, Port #2
        //changed sign to negative when encoder flipped - 1/17/21
        int currEnYLeft = mFL.getCurrentPosition() - encoderYLeftStart; // Motor Front Left & Odometer Y-Left, Port #0
        int currEnYRight = mBL.getCurrentPosition() - encoderYRightStart; // Motor Back Left & Odometer Y-Right, Port #3

        //original encoder statements for reference
        //int currEnYLeft = mFL.getCurrentPosition() - encoderYLeftStart; // Motor Front Left & Odometer Y-Left, Port #0
        //int currEnYRight = -mBL.getCurrentPosition() + encoderYRightStart; // Motor Back Left & Odometer Y-Right, Port #3

        double currGyro = -gyroZ;

        int changeEnYLeft = currEnYLeft - prevEnYLeft;
        int changeEnYRight = currEnYRight - prevEnYRight;
        double changeGyro = currGyro - prevGyro;

        //Get change in encoder values
        // Note: be careful, must get sign correct in next equation or robot center of rotation will be offset
        // from true center of robot.
        double xRotateGuess = (changeEnYLeft - changeEnYRight) * (-7/8.25); // multiply ratio of x/y odometer radii
        //* 0.448; //Constant = |avg enY| / enX (prev = 1.033)
        //0.1025
        //0.448
        //changed first number from -5.25 to -7 because of flipped x encoder - 1/17/21
        double changeEnXVirtual = (currEnX - prevEnX + xRotateGuess);
        //enXprediction += currEnX - prevEnX + xRotateGuess; //Only used to compare to 2]nd enX

        double changeEnXPhysical = (currEnX - prevEnX);
        double changeEnX = (changeEnXPhysical + changeEnXVirtual) / 2;
        double changeEnY = (changeEnYLeft + changeEnYRight) / 2  ;

        //Update odometer values (+, -)
        posY += (Math.cos(Math.toRadians(gyroZ)) * changeEnY) - (Math.sin(Math.toRadians(gyroZ)) * changeEnX);
        posX -= (Math.sin(Math.toRadians(gyroZ)) * changeEnY) + (Math.cos(Math.toRadians(gyroZ)) * changeEnX);
        //posY += (Math.cos(Math.toRadians(prevGyro + (changeGyro / 2))) * changeEnY) - (Math.sin(Math.toRadians(prevGyro + (changeGyro / 2))) * changeEnX);
        //posX -= (Math.sin(Math.toRadians(prevGyro + (changeGyro / 2))) * changeEnY) + (Math.cos(Math.toRadians(prevGyro + (changeGyro / 2))) * changeEnX);

        //Set pose variables in inches
        pose.x = ticksToInches((int) posX);
        pose.y = ticksToInches((int) posY);
        pose.theta = Math.toRadians(gyroZ);

        //Set previous values for next iteration
        prevEnX = currEnX;
        prevEnYLeft = currEnYLeft;
        prevEnYRight = currEnYRight;
        prevGyro = currGyro;
    }

    public void moveToPose(double targetX, double targetY, double targetTheta, double rotatePercent) {
        //code below assumes rotatePercent between 0 and 1; user input will be percent-DCP 11/28/20
        rotatePercent/=100;

        //Distance
        double distanceX = targetX - pose.x;
        double distanceY = targetY - pose.y;
        double distanceToTarget = Math.sqrt(Math.pow((distanceX), 2) + Math.pow(distanceY, 2));

        //Starting values
        if (!isStartRecorded) {
            distanceToTargetStart = distanceToTarget;
            thetaStart = gyroZ;
            isStartRecorded = true;
        }

        //Robot orientation vs distance to target (in degrees)
        if (distanceToTarget > distanceToTargetStart * (1 - rotatePercent)) {
            thetaRobot = (1 - (distanceToTarget - distanceToTargetStart * (1 - rotatePercent)) / (distanceToTargetStart * rotatePercent)) * (targetTheta - thetaStart) + thetaStart;
        } else {
            thetaRobot = targetTheta;
        }

        //Positive rotation = clockwise = decrease in theta
        double rotate = (1 / (1 + Math.pow(Math.E, -(0.06 * (gyroZ - thetaRobot))))) - 0.5;

        // double rotate = (1 / (1 + Math.pow(Math.E, -(0.13 * (gyroZ - thetaRobot))))) - 0.5;

        //Threshold values for motor power
        rotate = thresholdMotorPower(rotate, 0.1);

        //Powers
        // old value = 0.04
        // strafe is forward, forward is strafe for 2021 robot
        double aggressivenessStrafe = 0.025;
        double aggressivenessForward = 0.08; //0.04
        double PmaxStrafe = 1;
        double PmaxForward = 1;
        double strafe = PmaxStrafe * ((2 / (1 + Math.pow(Math.E, -(aggressivenessStrafe * (distanceToTarget * Math.cos(-pose.theta - Math.atan(distanceX / distanceY) + Math.toRadians(rotate * 20))))))) - 1);
        double forward = PmaxForward * ((2 / (1 + Math.pow(Math.E, -(aggressivenessForward * (distanceToTarget * Math.sin(-pose.theta - Math.atan(distanceX / distanceY) + Math.toRadians(rotate * 20))))))) - 1);

        //Threshold values for motor power
        forward = thresholdMotorPower(forward, 0.2); //0.25, 0.2, 0.1
        strafe = thresholdMotorPower(strafe, 0.15);

        //Adjust for quadrants
        if (pose.y < targetY) {
            //forward = -forward;
            strafe = -strafe;
        }
        else{
            forward = -forward;
        }

        drive(strafe, forward, -rotate);

        //  drive(strafe, forward, rotate);
    }

    public void moveToPoseWaypoint(double targetX, double targetY, double targetTheta, double rotatePercent) {

        //Distance
        double distanceX = (targetX - pose.x);
        double distanceY = (targetY - pose.y);
        double distanceToTarget = Math.sqrt(Math.pow((distanceX), 2) + Math.pow(distanceY, 2));

        //Starting values
        if (!isStartRecorded) {
            distanceToTargetStart = distanceToTarget;
            thetaStart = gyroZ;
            isStartRecorded = true;
        }

        //Powers
        double strafe = distanceToTarget * Math.cos(-pose.theta - Math.atan(distanceX / distanceY)) / 24;
        double forward = distanceToTarget * Math.sin(-pose.theta - Math.atan(distanceX / distanceY)) / 24;

        //Threshold values for motor power
        forward = thresholdMotorPower(forward, 0.5);
        strafe = thresholdMotorPower(strafe, 0.5);

        //Adjust for quadrants
        if (pose.y < targetY) {
            forward = -forward;
            strafe = -strafe;
        }

        //Robot orientation vs distance to target (in degrees)
        if (distanceToTarget > distanceToTargetStart * (1 - rotatePercent)) {
            thetaRobot = (1 - (distanceToTarget - distanceToTargetStart * (1 - rotatePercent)) / (distanceToTargetStart * rotatePercent)) * (targetTheta - thetaStart) + thetaStart;
        } else {
            thetaRobot = targetTheta;
        }

        //Positive rotation = clockwise = decrease in theta
        double rotate = (1 / (1 + Math.pow(Math.E, -(0.13 * (gyroZ - thetaRobot))))) - 0.5;

        //Threshold values for motor power
        rotate = thresholdMotorPower(rotate, 0.2);

        drive(forward, strafe, -rotate);
    }

    public boolean isInToleranceOLD(double targetX, double targetY, double toleranceDistance) {
        if (Math.abs(pose.x - targetX) < toleranceDistance && Math.abs(pose.y - targetY) < toleranceDistance) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isInTolerance(double targetX, double targetY, double targetTheta, double toleranceDistance, double toleranceTheta) {
        if (Math.abs(pose.x - targetX) < toleranceDistance && Math.abs(pose.y - targetY) < toleranceDistance && Math.abs(gyroZ - targetTheta) < toleranceTheta) {
            return true;
        } else {
            return false;
        }
    }


    public double thresholdMotorPower(double power, double threshold) {
        if (power > 1) {
            return 1;
        } else if (power < -1) {
            return -1;
        } else if (power < threshold && power > 0) {
            return threshold;
        } else if (power > -threshold && power < 0) {
            return -threshold;
        } else {
            return power;
        }
    }

    //Moves drive train in all possible directions
    public void drive(double forward, double strafe, double rotate) {


        mFL.setPower(forward + strafe + rotate);
        mFR.setPower(forward - strafe - rotate);
        mBL.setPower(forward - strafe + rotate);
        mBR.setPower(forward + strafe - rotate);
    }

    public void driveBackward(double power) {
        driveForward(-power);
    }

    public void driveForward(double power) {
        drive(power, 0, 0);
        return;
    }

    public void driveLeft(double power) {
        drive(0, -power, 0);
    }

    public void driveRight(double power) {
        driveLeft(-power);
    }

   /* public void gyroUpdate() {

        Orientation angles = imuExpansion.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float originalAngle = angles.firstAngle - lastAngle.firstAngle;
//        double angle1 = originalAngle;
//        double angle2 = angle1;
//        double angle3 = angle2;
//        double angle4 = angle3;
//        double angle5 = angle4;


        if (originalAngle < -180) {
            originalAngle += 360;
        } else if (originalAngle > 180) {
            originalAngle -= 360;
        }

        lastAngle = angles;
        gyroZ += originalAngle;
        //gyroZ += (originalAngle + angle2 + angle3 + angle4 + angle5)/5;

    }*/

    //Inverse sigmoid function for use in drivetrain directional movement
    public double inverseSigmoid(float input) {
        double value = Math.log((1 - input) / (input + 1)) / -5;
        if (value > 1) {
            value = 1;
        } else if (value < -1) {
            value = -1;
        } else {
            return value;
        }
        return value;
    }



    public void odometerUpdate() {

        float gyroEuclid = gyroZ % 360;

        if (gyroEuclid < 0) {
            gyroEuclid += 360;
        }
/*
        //Set current values for iteration
        currEnX = mFR.getCurrentPosition();
        currEnYLeft = -mFL.getCurrentPosition();

        //Get change in encoder values
        changeEnX = currEnX - prevEnX;
        changeEnYLeft = currEnYLeft - prevEnYLeft;
        changeDeg = gyroZ - prevDeg;

        //Adjust for rotation (measured constants are in ticks/degree)
        changeEnX = (int)(changeEnX - (changeDeg * -12.8877098));
        changeEnYLeft = (int)(changeEnYLeft - (changeDeg * -15.6436288));

        //Calculate multiplier for percent of encoder resulting in odometer values
        multiplier = ((2 - Math.sqrt(2)) / 4) * Math.cos((Math.PI / 180) * 4 * gyroEuclid) + ((2 + Math.sqrt(2)) / 4);

        //Adjust for quadrant
        if(gyroEuclid >= 45 && gyroEuclid < 135){
            changeOdX = multiplier * changeEnYLeft;
            changeOdY = multiplier * -changeEnX;
        } else if(gyroEuclid >= 135 && gyroEuclid < 225){
            changeOdX = multiplier * -changeEnX;
            changeOdY = multiplier * -changeEnYLeft;
        } else if(gyroEuclid >= 225 && gyroEuclid < 315){
            changeOdX = multiplier * -changeEnYLeft;
            changeOdY = multiplier * changeEnX;
        } else {
            changeOdX = multiplier * changeEnX;
            changeOdY = multiplier * changeEnYLeft;
        }


        //Update odometer values
        odX += changeOdX;
        odY += changeOdY;

        //Set previous values for next iteration
        prevEnX = currEnX;
        prevEnYLeft = currEnYLeft;
        prevDeg = gyroZ; */
    }

    public void rotateClockwise(double power) {
        drive(0, 0, power);
    }

    public void rotateCounterclockwise(double power) {
        rotateClockwise(-power);
    }

    public void stopDriveTrain() {
        drive(0, 0, 0);
    }

    public void moveToPoseTele(int targetX, int targetY, int forward, boolean drive, boolean rotateBeforeDrive) {

        double distanceX = (targetX - pose.x);
        double distanceY = (targetY - pose.y);
        double altSpeed;

        double tangentOf = (targetY - pose.y) / distanceX;
        double distanceToTarget = Math.sqrt(Math.pow((distanceX), 2) + Math.pow(distanceY, 2));

        double distanceToTheta = (Math.atan(tangentOf) - pose.theta);

        if (distanceToTheta > Math.PI / 2) {
            distanceToTheta -= Math.PI;
        }

        if (distanceToTheta < -Math.PI / 2) {
            distanceToTheta += Math.PI;
        }

        double distanceSpeed = (2 / (1 + Math.pow(Math.E, -0.03 * distanceToTarget))) - 1;

        if (Math.abs(Math.toDegrees(distanceToTheta)) > 30 && rotateBeforeDrive) {
            distanceSpeed = 0;
        }

        double leftSidePower = 0;
        double rightSidePower = 0;

        if (distanceToTheta < 0) {
            if (forward > 0) {
                leftSidePower = distanceSpeed * forward; // + orientationSpeed;
                rightSidePower = distanceSpeed * (Math.cos(distanceToTheta) / 6) * forward; // - orientationSpeed;
                //telemetry.addData("Left Side Dominates", "");
            } else {
                leftSidePower = distanceSpeed * (Math.cos(distanceToTheta) / 6) * forward; // - orientationSpeed;
                rightSidePower = distanceSpeed * forward; // + orientationSpeed;
                //telemetry.addData("Right Side Dominates", "");
            }
        } else {
            if (forward > 0) {
                leftSidePower = distanceSpeed * (Math.cos(distanceToTheta) / 6) * forward; // - orientationSpeed;
                rightSidePower = distanceSpeed * forward; // + orientationSpeed;
                //telemetry.addData("Right Side Dominates", "");
            } else {
                leftSidePower = distanceSpeed * forward; // + orientationSpeed;
                rightSidePower = distanceSpeed * (Math.cos(distanceToTheta) / 6) * forward; // - orientationSpeed;
                //telemetry.addData("Left Side Dominates", "");
            }
        }

        if (drive) {
            mFL.setPower(leftSidePower);
            mFR.setPower(rightSidePower);
            mBL.setPower(leftSidePower);
            mBR.setPower(rightSidePower);
        }
        /*
        telemetry.addData("Pose X", pose.x);
        telemetry.addData("Pose Y", pose.y);
        telemetry.addData("Distance to Theta", Math.toDegrees(distanceToTheta));
        telemetry.addData("Tan", Math.toDegrees(Math.atan(tangentOf)));
        telemetry.addData("Left Side Power", leftSidePower);
        telemetry.addData("Right Side Power", rightSidePower);
        telemetry.update();
        */
    }
}
