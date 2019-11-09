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
    private static long ytimeout = 0;
    private static long curtime;
    private static boolean switching=false;
    private static boolean fromheld=false;
    private static int intakeArmLastPos;

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

        robot.intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            boolean fulldrive = !gamepad1.x;
            boolean apressed = gamepad1.a;
            boolean leftBump = gamepad1.left_bumper;
            boolean ypressed = gamepad1.y;
            float intakeMult=1;
            if(leftBump) intakeMult = 0.5f;
            float rx = gamepad1.right_stick_x;
            float ry = gamepad1.right_stick_y;
            float lx = gamepad1.left_stick_x;
            float ly = gamepad1.left_stick_y;
            mult = map(mult, 0,1,minspeed,maxspeed);
            //down is 860
            //folded is 0
            if(ypressed && System.currentTimeMillis()-ytimeout > 250){
                robot.trayPullerLeft.setPosition(1-robot.trayPullerLeft.getPosition());
                robot.trayPullerRight.setPosition(1-robot.trayPullerRight.getPosition());
                ytimeout = System.currentTimeMillis();
            }
            if(fulldrive){
                robot.intakeArm.setPower(0);
                //robot.intakeArm.setTargetPosition(intakeArmLastPos);
                //robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //robot.intakeArm.setPower(1);
                drive.setMotorPower(-(rx)*mult, (ry)*mult, (lx)*mult, (ly)*mult, true);//if you change doFunction, make sure to also change it in RRBotAutoReader
            }else{
                drive.setMotorPower(-(rx)*mult, (ry)*mult, 0, 0, true);
                /*if(robot.intakeArm.getCurrentPosition()>=0 || robot.intakeArm.getCurrentPosition()<=860) {
                    if(ly!=0) robot.intakeArm.setTargetPosition(robot.intakeArm.getCurrentPosition() + 25*(int)(ly/Math.abs(ly)));
                    else robot.intakeArm.setTargetPosition(robot.intakeArm.getCurrentPosition());
                    robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.intakeArm.setPower(1);
                }
                intakeArmLastPos = robot.intakeArm.getCurrentPosition();*/
                robot.intakeArm.setPower(ly);
            }
            telemetry.addData("IntakeArmPos",robot.intakeArm.getCurrentPosition());
            telemetry.update();
            if(switching){
                if(System.currentTimeMillis()-curtime > 500){//time before applying reverse voltage to switch direction
                    robot.intakeMotorLeft.setPower(1*intakeMult);//should be mleming out
                    robot.intakeMotorRight.setPower(1*intakeMult);
                    switching=false;
                }
            }else {
                if (apressed && !timerstart && !fromheld) {
                    timerstart = true;
                    curtime = System.currentTimeMillis();
                } else if (timerstart && !fromheld) {
                    if (apressed) {
                        if (System.currentTimeMillis() - curtime > 300) {//delay before holding A turns into spew mode
                            if(robot.intakeMotorLeft.getPower()<0){
                                robot.intakeMotorLeft.setPower(0);
                                robot.intakeMotorRight.setPower(0);
                                switching = true;
                                fromheld=true;
                                curtime=System.currentTimeMillis();
                            }else if(robot.intakeMotorLeft.getPower()==0){
                                robot.intakeMotorLeft.setPower(1*intakeMult);
                                robot.intakeMotorRight.setPower(1*intakeMult);
                            }
                        }
                    } else {
                        if (robot.intakeMotorLeft.getPower() !=0) {
                            robot.intakeMotorLeft.setPower(0);
                            robot.intakeMotorRight.setPower(0);
                        } else if (robot.intakeMotorLeft.getPower() == 0) {
                            robot.intakeMotorLeft.setPower(-1*intakeMult);//should be slurping in
                            robot.intakeMotorRight.setPower(-1*intakeMult);
                        }
                        timerstart = false;
                    }
                }else if(!apressed){
                    fromheld=false;
                }
            }
        }
        else
        {
            drive.AutoMoveEndCheck();
        }
    }
}
