package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Base64;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;

@Autonomous(name="FullAutoRed")

public class FullAutoRed extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    RRBotHardware robot = new RRBotHardware();

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AVVIoiT/////AAABmX81rTPW60hcgKTP12YL9sFIHAXL7WyR1JI578v+YFJG/JSwjny6iEWiHEZ+twbt7HQ61pyg3A4/CCjpG1/u6VC6N2uK5bnWgFzeIHRESoUVX0pbphXVmkJ8NQmi9ZdKeNKV2ZgnM++ZT3cwvksRhXaA5LfVH0oB3XGNhrOzteP66UquAJUaNRKnMRjH4VjBiw9EWD1YGImGzeFPpA0p2xTKXQZAfLalNGnDRXM+3BlUfJsFbaSR+Uu/C3MIb8PMyA6h1nQGxMaIZLnl/Py2LPFgo5prafgdcD+9tV/BqE9F89AJC5LvHwOSKTfvsF9qe0fsZHFjg/+h10hZdeF8b1bQBhVO2OZf/T/e94I85MOh";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private static double orientation = 0; //Orientation of 0 means that the front of the robot is facing the midline of the field (throught the neutral bridge)
    private static double correctnessThreshold=1;
    private static double maxTime = 4000; //Milliseconds allotted for angle autocorrect
    private static float maxTurnPower = 1f;

    private static float xPosition;
    private static float yPosition;

    private static long startMS;

    //gyro variables
    private BNO055IMU imu;
    private Orientation angles;

    //Motor Definitions
    private final double COUNTS_PER_MOTOR_REV = 537.6;    // Andymark 20:1 gearmotor
    private final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    //construct drive class
    RRBotMecanumDrive drive = new RRBotMecanumDrive(robot);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);
        orientation = 0; //Starting Orientation
        initGyro();
        initVuforia();

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.rearRightMotor.getCurrentPosition(),
                robot.rearLeftMotor.getCurrentPosition(),
                robot.frontRightMotor.getCurrentPosition(),
                robot.frontLeftMotor.getCurrentPosition());
        telemetry.update();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (tfod != null) {
            tfod.activate();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        startMS = System.currentTimeMillis();
        while (opModeIsActive()) {

            EncoderDriveSideways(0.75,-8.5,10);//strafe 20 inches to the left

            EncoderDriveTank(1,-24,-24,10); //run to foundation
            EncoderDriveTank(Constants.autoSpeed,-10,-10,10); //run to foundation

            robot.trayPullerLeft.setPosition(0);//Grasp foundation with servos
            robot.trayPullerRight.setPosition(1);//^^^
            sleep(400);//Wait for servos
            EncoderDriveTank(1,24,24,10);//bring foundation back to wall
            turnAngle("right",90,0.75);
            robot.trayPullerLeft.setPosition(1);//Release servos
            robot.trayPullerRight.setPosition(0);//^^^
            sleep(400);//Wait for servos

            //Back up into corner for consistency
            EncoderDriveTank(1,-10,-10,5);

            EncoderDriveSideways(0.5,4,5);

            EncoderDriveTank(1,65,65,10);//forward 20 inches

            EncoderDriveSideways(0.75,-12,5);

            turnAngle("left",90);

            centerOnSkystone();

            requestOpModeStop();
        }
    }

    public Recognition findLeftmost(List<Recognition> recogs){
        Recognition leftmost = recogs.get(0);
        float leftmostpx=recogs.get(0).getImageWidth();
        for(Recognition recog : recogs){
            if(recog.getLabel().equals("Stone") || recog.getLabel().equals("Skystone")){
                if(recog.getLeft()<leftmostpx) {
                    leftmostpx = recog.getLeft();
                    leftmost = recog;
                }
            }
        }
        return leftmost;
    }
    public Recognition findSkystone(List<Recognition> recogs){
        float leftmostpx=1000f;
        Recognition leftmostskystone=null;
        for(Recognition recog : recogs){
            if(recog.getLabel().equals("Skystone")) {
                if(recog.getLeft()<leftmostpx){
                    leftmostpx = recog.getLeft();
                    leftmostskystone = recog;
                }
            }
        }
        if(leftmostskystone!=null) return leftmostskystone;
        else return null;
    }

    public void centerOnSkystone(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            int timeschecked=0;
            while(System.currentTimeMillis()-startMS < 29500){
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null && updatedRecognitions.size()>0) {
                    //telemetry.addData("# Object Detected", updatedRecognitions.size());
                    Recognition leftmost = findLeftmost(updatedRecognitions);
                    float pixels=1000000;
                    if(leftmost.getLabel().equals("Skystone")){
                        while(Math.abs(pixels)>200f){
                            List<Recognition> recogs = tfod.getUpdatedRecognitions();
                            if(recogs != null && recogs.size()>0 && findSkystone(recogs)!=null){
                                Recognition skystone = findSkystone(recogs);
                                pixels = (skystone.getImageWidth()-skystone.getRight()) - (skystone.getLeft());//positive means must go right (bot must go left)
                                EncoderDriveSideways(1,2*(pixels>0 ? 1 : -1),5);
                            }
                        }
                        break;
                    }else if(timeschecked>=3){
                        EncoderDriveSideways(0.5,-4,5);
                        timeschecked=0;
                    }else timeschecked++;
                }
            }
            AutoCorrect();
            EncoderDriveSideways(1,14,5);
            EncoderDriveTank(1,-32,-32,5);
            EncoderDriveTank(1,8,8,5);
            EncoderDriveSideways(1,6,5);
            turnAngle("right",90);
            flipOutLift();
            EncoderDriveSideways(0.5,5,5);
            EncoderDriveTank(0.5,-5,-5,5);
            EncoderDriveSideways(0.5,-5,5);
            robot.blockGrabber.setPosition(1);
            sleep(250);
            EncoderDriveSideways(0.5,-16,5);
            EncoderDriveTank(1,-50,-50,5);
            robot.blockGrabber.setPosition(0);
            EncoderDriveTank(1,25,25,5);

        }else{
            telemetry.addData(">>","TFOD is null");
            telemetry.update();
        }
    }

    public static float map(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public void flipOutLift(){
        robot.liftMotor.setTargetPosition(RRBotTeleop.liftClearanceHeight);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(0.75);
        long timer=System.currentTimeMillis();
        while(System.currentTimeMillis()-timer<1000) {}
        robot.liftMotor.setPower(0);
        robot.grabberArm.setPosition(1-robot.grabberArm.getPosition());
        timer=System.currentTimeMillis();
        while(System.currentTimeMillis()-timer<750){}
        robot.liftMotor.setTargetPosition(0);
        robot.liftMotor.setPower(0.75);
        timer=System.currentTimeMillis();
        while(System.currentTimeMillis()-timer<1250) {}
        robot.liftMotor.setPower(0);
    }

    public void AutoCorrect(double power){
        double prevPowerRR = robot.rearRightMotor.getPower();
        double prevPowerRL = robot.rearLeftMotor.getPower();
        double prevPowerFR = robot.frontRightMotor.getPower();
        double prevPowerFL = robot.frontLeftMotor.getPower();
        int dirTurn = 1;
        double traveled=0;
        double leftmost=0;
        double rightmost=0;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float startHeading = angles.firstAngle;
        long doneTime = 0;
        boolean timerStart=false;
        long startTime = System.currentTimeMillis();
        while((!timerStart || Math.abs(startHeading-orientation)>correctnessThreshold || System.currentTimeMillis()-doneTime<100) && System.currentTimeMillis()-startTime < maxTime && System.currentTimeMillis()-startMS<29500){
            if(Math.abs(startHeading-orientation)<=correctnessThreshold && !timerStart){
                doneTime=System.currentTimeMillis();
                timerStart=true;
            }else if(Math.abs(startHeading-orientation)>correctnessThreshold){
                doneTime=0;
                timerStart=false;
            }

            startHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if(startHeading>orientation && startHeading-orientation<180) dirTurn=-1;
            else if(startHeading<orientation && orientation-startHeading>180) dirTurn=-1;
            else dirTurn=1;
            leftmost = startHeading>orientation ? startHeading : orientation;
            rightmost = startHeading+orientation-leftmost;
            if(leftmost<0 != rightmost<0){
                traveled = leftmost+Math.abs(rightmost)<90 ? leftmost+Math.abs(rightmost) : (180-leftmost)+180-Math.abs(rightmost);
            }else{
                traveled = leftmost>0 ? leftmost-rightmost : Math.abs(rightmost-leftmost);
            }
            double turnPower = map((float)traveled, 0,30,maxTurnPower/10,maxTurnPower);
            if(turnPower>maxTurnPower) turnPower=maxTurnPower;
            robot.rearRightMotor.setPower(prevPowerRR+(turnPower*dirTurn*power));
            robot.rearLeftMotor.setPower(prevPowerRL+(-turnPower*dirTurn*power));
            robot.frontRightMotor.setPower(prevPowerFR+(turnPower*dirTurn*power));
            robot.frontLeftMotor.setPower(prevPowerFL+(-turnPower*dirTurn*power));
        }
        robot.rearRightMotor.setPower(prevPowerRR);
        robot.rearLeftMotor.setPower(prevPowerRL);
        robot.frontRightMotor.setPower(prevPowerFR);
        robot.frontLeftMotor.setPower(prevPowerFL);
    }

    public void AutoCorrect(){
        double prevPowerRR = robot.rearRightMotor.getPower();
        double prevPowerRL = robot.rearLeftMotor.getPower();
        double prevPowerFR = robot.frontRightMotor.getPower();
        double prevPowerFL = robot.frontLeftMotor.getPower();
        int dirTurn = 1;
        double traveled=0;
        double leftmost=0;
        double rightmost=0;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float startHeading = angles.firstAngle;
        long doneTime = 0;
        boolean timerStart=false;
        long startTime = System.currentTimeMillis();
        while((!timerStart || Math.abs(startHeading-orientation)>correctnessThreshold || System.currentTimeMillis()-doneTime<100) && System.currentTimeMillis()-startTime < maxTime && System.currentTimeMillis()-startMS<29500){
            if(Math.abs(startHeading-orientation)<=correctnessThreshold && !timerStart){
                doneTime=System.currentTimeMillis();
                timerStart=true;
            }else if(Math.abs(startHeading-orientation)>correctnessThreshold){
                doneTime=0;
                timerStart=false;
            }

            startHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if(startHeading>orientation && startHeading-orientation<180) dirTurn=-1;
            else if(startHeading<orientation && orientation-startHeading>180) dirTurn=-1;
            else dirTurn=1;
            leftmost = startHeading>orientation ? startHeading : orientation;
            rightmost = startHeading+orientation-leftmost;
            if(leftmost<0 != rightmost<0){
                traveled = leftmost+Math.abs(rightmost)<90 ? leftmost+Math.abs(rightmost) : (180-leftmost)+180-Math.abs(rightmost);
            }else{
                traveled = leftmost>0 ? leftmost-rightmost : Math.abs(rightmost-leftmost);
            }
            double turnPower = map((float)traveled, 0,30,maxTurnPower/10,maxTurnPower);
            if(turnPower>maxTurnPower) turnPower=maxTurnPower;
            robot.rearRightMotor.setPower(prevPowerRR+(turnPower*dirTurn));
            robot.rearLeftMotor.setPower(prevPowerRL+(-turnPower*dirTurn));
            robot.frontRightMotor.setPower(prevPowerFR+(turnPower*dirTurn));
            robot.frontLeftMotor.setPower(prevPowerFL+(-turnPower*dirTurn));
        }
        robot.rearRightMotor.setPower(prevPowerRR);
        robot.rearLeftMotor.setPower(prevPowerRL);
        robot.frontRightMotor.setPower(prevPowerFR);
        robot.frontLeftMotor.setPower(prevPowerFL);
    }

    public void DriveDirection(double speed, double angle, double timeoutS){ //0 Degrees is strafing directly to the right
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double jx = Math.cos(Math.toRadians(angle));
        double jy = Math.sin(Math.toRadians(angle));

        if (opModeIsActive())
        {
            runtime.reset();
            drive.setMotorPower(jx*speed,jy*speed,0,0,true);
            while(opModeIsActive() &&
                    (runtime.seconds() < timeoutS))
            {
                AutoCorrect();
            }

            TurnOffMotors();

            // Turn off RUN_TO_POSITION
            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public void EncoderDriveTank(double speed, double leftInches, double rightInches, double timeoutS)
    {
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newRearLeftTarget;
        int newRearRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            newRearLeftTarget = robot.rearLeftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRearRightTarget = robot.rearRightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.rearLeftMotor.setTargetPosition(newRearLeftTarget);
            robot.rearRightMotor.setTargetPosition(newRearRightTarget);
            robot.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.frontRightMotor.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.rearLeftMotor.setPower(Math.abs(speed));
            robot.rearRightMotor.setPower(Math.abs(speed));
            robot.frontLeftMotor.setPower(Math.abs(speed));
            robot.frontRightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while(opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy() && robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()))
            {
                AutoCorrect();
            }

            TurnOffMotors();

            // Turn off RUN_TO_POSITION
            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setAngle(double orientation){
        this.orientation = orientation;
        AutoCorrect();
    }
    public void turnAngle(String direction, double angle){
        if(direction=="left") this.orientation += angle;
        else if(direction=="right") this.orientation -= angle;
        AutoCorrect();
    }public void turnAngle(String direction, double angle, double power){
        if(direction=="left") this.orientation += angle;
        else if(direction=="right") this.orientation -= angle;
        AutoCorrect(power);
    }


    public void TurnByGyro(String direction, int angle, double speed)
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float startHeading = angles.firstAngle;

        if(direction.equals("left"))
        {
            robot.rearRightMotor.setPower(speed);
            robot.rearLeftMotor.setPower(-speed);
            robot.frontRightMotor.setPower(speed);
            robot.frontLeftMotor.setPower(-speed);
        }
        else if(direction.equals("right"))
        {
            robot.rearRightMotor.setPower(-speed);
            robot.rearLeftMotor.setPower(speed);
            robot.frontRightMotor.setPower(-speed);
            robot.frontLeftMotor.setPower(speed);
        }

        while(opModeIsActive() && Math.abs(angles.firstAngle - startHeading) < angle)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("heading", formatAngle(AngleUnit.DEGREES, angles.firstAngle));
            telemetry.update();
        }

        TurnOffMotors();
    }
    public void initGyro()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void EncoderDriveSideways(double speed, double distance, double timeoutS)
    {
        //reset encoders
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newRearLeftTarget;
        int newRearRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        //ensure that the opmode is still active
        if(opModeIsActive())
        {
            //calculate target positions, negative for two motors so the robot strafes
            newRearLeftTarget = robot.rearLeftMotor.getCurrentPosition() + (int) (-distance * Math.sqrt(2) * COUNTS_PER_INCH);
            newRearRightTarget = robot.rearRightMotor.getCurrentPosition() + (int) (distance * Math.sqrt(2) * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int) (distance * Math.sqrt(2) * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int) (-distance * Math.sqrt(2) * COUNTS_PER_INCH);
            robot.rearLeftMotor.setTargetPosition(newRearLeftTarget);
            robot.rearRightMotor.setTargetPosition(newRearRightTarget);
            robot.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.frontRightMotor.setTargetPosition(newFrontRightTarget);

            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            robot.rearLeftMotor.setPower(Math.abs(speed));
            robot.rearRightMotor.setPower(Math.abs(speed));
            robot.frontLeftMotor.setPower(Math.abs(speed));
            robot.frontRightMotor.setPower(Math.abs(speed));

            //keep looping until one of the motors finished its movement
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy() && robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()))
            {
                AutoCorrect();
            }

            TurnOffMotors();

            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    public void TurnOffMotors()
    {
        robot.rearRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
    }
}
