package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Locale;

@Autonomous(name="RRBotAutoReader")

public class RRBotAutoReader extends LinearOpMode{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    RRBotHardware robot = new RRBotHardware();

    BufferedReader br;
    private static boolean timerstart=false;
    private static long curtime;
    private static boolean switching=false;

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

        try{
            br = new BufferedReader(new FileReader(new File("res/teleop_rec")));//MIGHT NEED TO FIX THIS FILENAME
        }catch(FileNotFoundException e) {e.printStackTrace();}

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

        String curline;
        // run until the end of the match (driver presses STOP)
        try{
            while (opModeIsActive() && (curline = br.readLine()) !=null) {
                String[] values = curline.split(",");
                float mult = Float.parseFloat(values[5]);
                mult = RRBotTeleop.map(mult, 0,1,RRBotTeleop.minspeed,RRBotTeleop.maxspeed);
                drive.setMotorPower((Float.parseFloat(values[2]))*mult, (Float.parseFloat(values[3]))*mult, (Float.parseFloat(values[0]))*mult, (Float.parseFloat(values[0]))*mult, false);

                boolean apressed = Boolean.parseBoolean(values[4]);
                if(switching){
                    if(System.currentTimeMillis()-curtime > 500){//time before applying reverse voltage to switch direction
                        robot.intakeMotorLeft.setPower(-1);//should be mleming out
                        robot.intakeMotorRight.setPower(-1);
                        switching=false;
                    }
                }else {
                    if (apressed && !timerstart) {
                        timerstart = true;
                        curtime = System.currentTimeMillis();
                    } else if (timerstart) {
                        if (apressed) {
                            if (System.nanoTime() - curtime > 500) {//delay before holding A turns into spew mode
                                robot.intakeMotorLeft.setPower(0);
                                robot.intakeMotorRight.setPower(0);
                                switching = true;
                                curtime=System.currentTimeMillis();
                            }
                        } else {
                            if (robot.intakeMotorLeft.getPower() == 1 && robot.intakeMotorLeft.getPower() == -1) {
                                robot.intakeMotorLeft.setPower(0);
                                robot.intakeMotorRight.setPower(0);
                            } else if (robot.intakeMotorLeft.getPower() == 0) {
                                robot.intakeMotorLeft.setPower(1);//should be slurping in
                                robot.intakeMotorRight.setPower(1);
                            }
                            timerstart = false;
                        }
                    }
                }
            }
        }catch(IOException e){e.printStackTrace();}

    }
}
