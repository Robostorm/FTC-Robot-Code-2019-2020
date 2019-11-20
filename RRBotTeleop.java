package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Misc.RRBotHardware;
import org.firstinspires.ftc.teamcode.Misc.RRBotMecanumDrive;

/**
 * TeleOp class, contains separate functions that run the robot during driver operated period of the game
 * @author John Brereton
 * @since 9/8/2019
 */

@TeleOp(name="RRBotTeleop")
public class RRBotTeleop extends OpMode
{
    public static float minspeed = 0.5f; // with gas pedal not pressed
    public static float maxspeed = 1.5f; // with gas pedal at full

    private static boolean timerstart=false;
    private static long ytimeout = 0;
    private static long btimeout = 0;
    private static long throwbackTime = 0;
    private static long throwdownTime = 0;
    private static long curtime;
    private static boolean inverted=false;
    private static long rbumptimer=0;
    private static boolean switching=false;
    private static boolean midthrowback=false;
    private static boolean midthrowdown=false;
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

        //robot.intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            //boolean leftBump = gamepad1.left_bumper;
            boolean ypressed = gamepad1.y;
            boolean bpressed = gamepad1.b;
            boolean dpadup = gamepad1.dpad_up;
            boolean dpaddown = gamepad1.dpad_down;
            boolean lbump = gamepad1.left_bumper;
            float rx = gamepad1.right_stick_x;
            float ry = gamepad1.right_stick_y;
            float lx = gamepad1.left_stick_x;
            float ly = gamepad1.left_stick_y;
            boolean rbump = gamepad1.right_bumper;
            int invmult = 1;
            mult = map(mult, 0,1,minspeed,maxspeed);
            if(rbump && System.currentTimeMillis()-rbumptimer > 500){
                inverted = !inverted;
                rbumptimer = System.currentTimeMillis();
            }
            if(inverted) invmult=-1;
            mult*=invmult;
            if(dpadup && System.currentTimeMillis()-throwbackTime > 500 && !midthrowdown) {
                throwbackTime = System.currentTimeMillis();
                robot.intakeArm.setPower(0.6);
                midthrowback = true;
            }
            if(dpaddown && System.currentTimeMillis()-throwdownTime > 500 && !midthrowback){
                throwdownTime = System.currentTimeMillis();
                robot.intakeArm.setPower(-0.6);
                midthrowdown=true;
            }
            if(midthrowback && fulldrive && robot.intakeArm.getPower()>0 && (System.currentTimeMillis()-throwbackTime>1000)){
                robot.intakeArm.setPower(0);
                midthrowback=false;
                throwbackTime=0;
            }
            if(midthrowdown && fulldrive && robot.intakeArm.getPower()<0 && (System.currentTimeMillis()-throwdownTime>1000)){
                robot.intakeArm.setPower(0);
                midthrowdown=false;
                throwdownTime=0;
            }
            if(bpressed && System.currentTimeMillis()-btimeout > 500){
                robot.blockGrabber.setPosition(1-(robot.blockGrabber.getPosition()));
                btimeout = System.currentTimeMillis();

            }
            if(ypressed && System.currentTimeMillis()-ytimeout > 250){
                robot.trayPullerLeft.setPosition(1-robot.trayPullerLeft.getPosition());
                robot.trayPullerRight.setPosition(1-robot.trayPullerRight.getPosition());
                ytimeout = System.currentTimeMillis();
            }
            if(fulldrive && !midthrowback && !midthrowdown){
                robot.intakeArm.setPower(0);
<<<<<<< HEAD:OpModes/RRBotTeleop.java
                if(lbump){
                    drive.setMotorPower(rx*mult, -ry*mult, 0, 0, true);
                    robot.liftMotor.setPower(ly*0.25);
                }else {
                    robot.liftMotor.setPower(0);
                    drive.setMotorPower((rx)*mult, -(ry)*mult, (lx)*Math.abs(mult), -(ly)*mult, true);//if you change doFunction, make sure to also change it in RRBotAutoReader
=======
                //robot.intakeArm.setTargetPosition(intakeArmLastPos);
                //robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //robot.intakeArm.setPower(1);
                drive.setMotorPower(-(rx)*mult, (ry)*mult, (lx)*mult, (ly)*mult, false);//if you change doFunction, make sure to also change it in RRBotAutoReader
            }else{
                drive.setMotorPower(-(rx)*mult, (ry)*mult, 0, 0, false);
                /*if(robot.intakeArm.getCurrentPosition()>=0 || robot.intakeArm.getCurrentPosition()<=860) {
                    if(ly!=0) robot.intakeArm.setTargetPosition(robot.intakeArm.getCurrentPosition() + 25*(int)(ly/Math.abs(ly)));
                    else robot.intakeArm.setTargetPosition(robot.intakeArm.getCurrentPosition());
                    robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.intakeArm.setPower(1);
>>>>>>> parent of b67c0b4... Added Auto Class:RRBotTeleop.java
                }
            }else {
                drive.setMotorPower((rx)*mult, -(ry)*mult, 0, 0, true);
                if(!midthrowback && !midthrowdown) robot.intakeArm.setPower(ly);
            }
            //telemetry.addData("IntakeArmPos",robot.intakeArm.getCurrentPosition());
            telemetry.update();
            if(switching){
                if(System.currentTimeMillis()-curtime > 500){//time before applying reverse voltage to switch direction
                    robot.intakeMotorLeft.setPower(0.25);//should be mleming out
                    robot.intakeMotorRight.setPower(0.25);
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
                                robot.intakeMotorLeft.setPower(0.25);
                                robot.intakeMotorRight.setPower(0.25);
                            }
                        }
                    } else {
                        if (robot.intakeMotorLeft.getPower() !=0) {
                            robot.intakeMotorLeft.setPower(0);
                            robot.intakeMotorRight.setPower(0);
                        } else if (robot.intakeMotorLeft.getPower() == 0) {
                            robot.intakeMotorLeft.setPower(-1);//should be slurping in
                            robot.intakeMotorRight.setPower(-1);
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
