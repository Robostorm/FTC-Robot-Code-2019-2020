package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Misc.MovementVars;
import org.firstinspires.ftc.teamcode.Misc.RRBotHardware;
import org.firstinspires.ftc.teamcode.Misc.RRBotMecanumDrive;

import java.util.Base64;
import java.util.Locale;

@Autonomous(name="RRBotAuto")

public class RRBotAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    RRBotHardware robot = new RRBotHardware();

    public static final float autoSpeed = 0.25f;

    //gyro variables
    private BNO055IMU imu;
    private Orientation angles;

    //Motor Definitions
    private final double COUNTS_PER_MOTOR_REV = 537.6;    // Andymark 20:1 gearmotor
    private final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private final double WHEEL_DIAMETER_CM = 4.0 * 2.54;     // For figuring circumference
    private final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);


    //construct drive class
    RRBotMecanumDrive drive = new RRBotMecanumDrive(robot);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        long lastUpdateTime=0;
        //ArrayList<CurvePoint> allPoints = new ArrayList<>();
        //allPoints.add(new CurvePoint(0,50,1,1,50,Math.toRadians(50),1));
        //allPoints.add(new CurvePoint(50,50,1,1,50,Math.toRadians(50),1));
        while (opModeIsActive()) {
            /*RRBotMecanumDrive.followCurve(allPoints,Math.toRadians(90));
            long currTime = System.currentTimeMillis();
            if(currTime - lastUpdateTime >= 16){
                lastUpdateTime = currTime;
                drive.ApplyMovement();
            }*/
            /*MovementVars.movement_x = 25;
            MovementVars.movement_y = 50;
            MovementVars.movement_turn = 0;
            drive.ApplyMovement();
            sleep(5000);
            requestOpModeStop();*/
            EncoderDriveTank(1,10,10,10);
            requestOpModeStop();
            //telemetry.addData("Gyro Pos:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            //telemetry.update();
        }
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

    public double[] calcVelocities(double leftX, double leftY)
    {
        double moveY1 = leftY;
        double turn = leftX;

        //remap input values using a function
        /*if(doFunction)
        {
            moveX = inputFunction(moveX);
            moveY1 = inputFunction(moveY1);
            turn = inputFunction(turn);
            moveY2 = inputFunction(moveY2);
        }*/

        double v1 = moveY1 + turn;
        double v2 = moveY1 - turn;
        double v3 = moveY1 - turn;
        double v4 = moveY1 + turn;

        double max = Math.abs(v1);
        if(Math.abs(v2) > max)
            max = Math.abs(v2);
        if(Math.abs(v3) > max)
            max = Math.abs(v3);
        if(Math.abs(v4) > max)
            max = Math.abs(v4);
        if(max > 1)
        {
            v1 /= max;
            v2 /= max;
            v3 /= max;
            v4 /= max;
        }

        double[] velocities = {v1, v2, v3, v4};
        return velocities;
    }

    public void setMotorPower(double x, double y)
    {
        //calculate the velocities
        double[] velocities = calcVelocities(x, y);

        //set the motor power
        robot.frontLeftMotor.setPower(velocities[0]);
        robot.frontRightMotor.setPower(velocities[1]);
        robot.rearLeftMotor.setPower(velocities[2]);
        robot.rearRightMotor.setPower(velocities[3]);
    }

    //make another version of the method where you give it inches sideways and inches forward and it goes in a straight line there
    public void DriveDirection(double speed, double angle, double inches, double timeoutS){ //0 Degrees is strafing directly to the right
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sin(x−(1/4)π) - front right & back left (power, -1 to 1 where negative is back and positive is forward)
        //sin(x+(1/4)π) - front left & back right
        //double inchesfwd = inches*Math.sin(angle);

        //int rearLeftTarget = robot.rearLeftMotor.getCurrentPosition() + (int) (inchesfwd * COUNTS_PER_INCH);
        //int rearRightTarget = robot.rearRightMotor.getCurrentPosition() + (int) (inchesfwd * COUNTS_PER_INCH);
        //int frontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int) (inchesfwd * COUNTS_PER_INCH);
        //int frontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int) (inchesfwd * COUNTS_PER_INCH);

        double jx = Math.cos(angle);
        double jy = Math.sin(angle);

        if (opModeIsActive())
        {
            /*rearLeftTarget *= Math.abs(rearLeftPower);
            rearRightTarget *= Math.abs(rearRightPower);
            frontLeftTarget *= Math.abs(frontLeftPower);
            frontRightTarget *= Math.abs(frontRightPower);*/

            // reset the timeout time and start motion.

            // Turn On RUN_TO_POSITION
            /*robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

            runtime.reset();
            //public void setMotorPower(double x, double y)
            setMotorPower(jx*speed,jy*speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while(opModeIsActive() &&
                    (runtime.seconds() < timeoutS))
            {
                // Display it for the driver.
                //telemetry.addData("Path1", "Running to %7d :%7d", newRearLeftTarget, newRearRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.rearLeftMotor.getCurrentPosition(),
                        robot.rearRightMotor.getCurrentPosition(),
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition());
                telemetry.update();
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
            newRearLeftTarget = robot.rearLeftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_CM);
            newRearRightTarget = robot.rearRightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_CM);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_CM);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_CM);
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

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newRearLeftTarget, newRearRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.rearLeftMotor.getCurrentPosition(),
                        robot.rearRightMotor.getCurrentPosition(),
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition());
                telemetry.update();
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
