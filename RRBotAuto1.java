package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RRBotHardware;
import org.firstinspires.ftc.teamcode.RRBotMecanumDrive;

import java.util.Locale;

@Autonomous(name="BlueRepoPark")

public class RRBotAuto1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    RRBotHardware robot = new RRBotHardware();

    //gyro variables
    private BNO055IMU imu;
    private Orientation angles;

    //Motor Definitions
    private final double COUNTS_PER_MOTOR_REV = 537.6;    // Andymark 20:1 gearmotor
    private final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private static double orientation = 0; //Orientation of 0 means that the front of the robot is facing the midline of the field (throught the neutral bridge)
    private static double correctnessThreshold=1;
    private static double maxTime = 4000; //Milliseconds allotted for angle autocorrect
    private static float maxTurnPower = 1f;

    private static long startMS;

    //construct drive class
    RRBotMecanumDrive drive = new RRBotMecanumDrive(robot);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);
        initGyro();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.rearRightMotor.getCurrentPosition(),
                robot.rearLeftMotor.getCurrentPosition(),
                robot.frontRightMotor.getCurrentPosition(),
                robot.frontLeftMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        startMS = System.currentTimeMillis();
        while (opModeIsActive()) {
            // Show the elapsed game time and wheel power.
            /*telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();*/
            EncoderDriveSideways(Constants.autoSpeed,8.5,10);//strafe 20 inches to the left

            EncoderDriveTank(Constants.autoSpeed,-33,-33,10); //run to foundation
            robot.trayPullerLeft.setPosition(0);//Grasp foundation with servos
            robot.trayPullerRight.setPosition(1);//^^^
            sleep(800);//Wait for servos
            EncoderDriveTank(Constants.autoSpeed,32,32,10);//bring foundation back to wall
            robot.trayPullerLeft.setPosition(1);//Release servos
            robot.trayPullerRight.setPosition(0);//^^^
            sleep(800);//Wait for servos

            EncoderDriveSideways(Constants.autoSpeed,-24,10);//strafe 20 inches to the left

            EncoderDriveTank(Constants.autoSpeed,-20,-20,10);//forward 20 inches

            EncoderDriveSideways(Constants.autoSpeed,-16,10);//strafe 40 inches to the left, towards Audience

            requestOpModeStop();
        }
    }

    public static float map(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
                //AutoCorrect();
            }

            TurnOffMotors();

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
                //AutoCorrect();
            }

            TurnOffMotors();

            // Turn off RUN_TO_POSITION
            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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
    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public void TurnOffMotors()
    {
        robot.rearRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
    }
}
