package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    public DcMotor intakeMotorLeft = null;
    public DcMotor intakeMotorRight = null;
    public DcMotor intakeArm = null;
    public DcMotor liftMotor = null;

    public Servo trayPullerLeft = null;
    public Servo trayPullerRight = null;
    public Servo blockGrabber = null;
    public Servo grabberArm = null;
    public Servo endCapArm = null;

    //public DigitalChannel liftStartSwitch = null;

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
        rearRightMotor = hwMap.dcMotor.get("rear_right");
        rearLeftMotor = hwMap.dcMotor.get("rear_left");
        frontRightMotor = hwMap.dcMotor.get("front_right");
        frontLeftMotor = hwMap.dcMotor.get("front_left");
        intakeMotorLeft = hwMap.dcMotor.get("intake1");
        intakeMotorRight = hwMap.dcMotor.get("intake2");
        intakeArm = hwMap.dcMotor.get("intakeArm");
        liftMotor = hwMap.dcMotor.get("liftMotor");

        // Define and Initialize Servos
        trayPullerLeft = hwMap.servo.get("tray_puller_left");
        trayPullerRight = hwMap.servo.get("tray_puller_right");
        blockGrabber = hwMap.servo.get("block_grabber");
        grabberArm = hwMap.servo.get("grabber_arm");
        endCapArm = hwMap.servo.get("end_cap_arm");

        // Define and Initialize Others
        //liftStartSwitch = hwMap.digitalChannel.get("leftStartSwitch");

        // Set Position of Servos
        trayPullerLeft.setPosition(1);//up
        trayPullerRight.setPosition(0);//up
        grabberArm.setPosition(0);
        blockGrabber.setPosition(0);
        endCapArm.setPosition(0);

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
        intakeMotorLeft.setPower(0);
        intakeMotorRight.setPower(0);
        intakeArm.setPower(0);
        liftMotor.setPower(0);

        // Set all Digital Channels to Input
        //liftStartSwitch.setMode(DigitalChannel.Mode.INPUT);

        // Set drive motors to run using encoders
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets motors to brake mode
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}

