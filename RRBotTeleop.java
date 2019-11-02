package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp class, contains separate functions that run the robot during driver operated period of the game
 * @author John Brereton
 * @since 9/8/2019
 */

@TeleOp(name="RRBotTeleop")
public class RRBotTeleop extends OpMode
{
    public static float minspeed = 0.75f; // with gas pedal not pressed
    public static float maxspeed = 1.5f; // with gas pedal at full

    private static boolean timerstart=false;
    private static long curtime;
    private static boolean switching=false;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    RRBotHardware robot = new RRBotHardware();

    //construct drive class
    RRBotMecanumDrive drive = new RRBotMecanumDrive(robot);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robot.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        DriveUpdate();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public static float map(float x, float in_min, float in_max, float out_min, float out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public void DriveUpdate()
    {
        //if the robot is not driving automatically, set motor power to the manual drive algorithm based on gamepad inputs
        if(!drive.getIsAutoMove())
        {
            //drive.setMotorPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.right_stick_y, true);
            float mult = gamepad1.right_trigger;
            mult = map(mult, 0,1,minspeed,maxspeed);
            //drive.setMotorPower((gamepad1.right_stick_x)*mult, -(gamepad1.right_stick_y)*mult, (gamepad1.left_stick_x)*mult, -(gamepad1.left_stick_y)*mult, true);
            drive.setMotorPower((gamepad1.right_stick_x)*mult, (gamepad1.right_stick_y)*mult, (gamepad1.left_stick_x)*mult, (gamepad1.left_stick_y)*mult, false);//if you change doFunction, make sure to also change it in RRBotAutoReader

            //FIGURE THIS OUT vv
            boolean apressed = gamepad1.a;
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
        else
        {
            drive.AutoMoveEndCheck();
        }
    }
}
