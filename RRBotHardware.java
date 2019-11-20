package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RRBotHardware
{
    public static double worldXPosition = 0;
    public static double worldYPosition = 0;
    public static double worldAngle_rad = 0;

    /* Public OpMode members. */
    public DcMotor rearRightMotor = null;
    public DcMotor rearLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor frontLeftMotor = null;

    public Servo trayPullerLeft = null;
    public Servo trayPullerRight = null;
    public Servo blockGrabber = null;
    //public Servo pinPuller = null;

    public DcMotor intakeMotorLeft = null;
    public DcMotor intakeMotorRight = null;
    public DcMotor intakeArm = null;

    public DcMotor liftMotor = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public RRBotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        try {

            rearRightMotor = hwMap.dcMotor.get("rear_right");
            rearLeftMotor = hwMap.dcMotor.get("rear_left");
            frontRightMotor = hwMap.dcMotor.get("front_right");
            frontLeftMotor = hwMap.dcMotor.get("front_left");

            // Define and Initialize Servos
            trayPullerLeft = hwMap.servo.get("tray_puller_left");
            trayPullerRight = hwMap.servo.get("tray_puller_right");
            blockGrabber = hwMap.servo.get("block_grabber");
            //pinPuller = hwMap.servo.get("pin_puller");

            intakeMotorLeft = hwMap.dcMotor.get("intake1");
            intakeMotorRight = hwMap.dcMotor.get("intake2");
            intakeArm = hwMap.dcMotor.get("intakeArm");

            liftMotor = hwMap.dcMotor.get("liftMotor");
        }catch(Exception e){

        }

        // Set Position of Servos
        trayPullerLeft.setPosition(1);
        trayPullerRight.setPosition(0);
        blockGrabber.setPosition(1);

        //set motors to drive forwards
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        rearRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        intakeArm.setPower(0);
        liftMotor.setPower(0);

        // Set drive motors to run using encoders
        try {
            rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //intakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //intakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //intakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }catch(Exception e){

        }

        // Sets motors to brake mode
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intakeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intakeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
 }

