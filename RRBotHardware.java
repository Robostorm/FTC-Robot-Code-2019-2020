package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RRBotHardware
{
    /* Public OpMode members. */
    public DcMotor rearRightMotor = null;
    public DcMotor rearLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor frontLeftMotor = null;
    public DcMotor intake1;
    public DcMotor intake2;

    public Servo trayPullerLeft = null;
    public Servo trayPullerRight = null;
    public Servo pinPuller = null;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public RRBotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        rearRightMotor = hwMap.dcMotor.get("rear_right");
        rearLeftMotor = hwMap.dcMotor.get("rear_left");
        frontRightMotor = hwMap.dcMotor.get("front_right");
        frontLeftMotor = hwMap.dcMotor.get("front_left");
        intake1 = hwMap.dcMotor.get("intake1");
        intake2 = hwMap.dcMotor.get("intake2");


        // Define and Initialize Servos
        trayPullerLeft = hwMap.servo.get("tray_puller_left");
        trayPullerRight = hwMap.servo.get("tray_puller_right");
        //pinPuller = hwMap.servo.get("pin_puller");

        // Set Position of Servos
        trayPullerLeft.setPosition(1);
        trayPullerRight.setPosition(0);

        //set motors to drive forwards
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        rearRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        intake1.setPower(0);
        intake2.setPower(0);

        // Set drive motors to run using encoders
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Sets motors to brake mode
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos

    }
 }

