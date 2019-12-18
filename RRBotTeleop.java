package org.firstinspires.ftc.teamcode;

import android.renderscript.ScriptIntrinsicYuvToRGB;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TeleOp class, contains separate functions that run the robot during driver operated period of the game
 * @author Aidan Ferry
 * @since 9/8/2019
 */

@TeleOp(name="RRBotTeleop")
public class RRBotTeleop extends OpMode
{
    public static float minspeed = 0.5f; // with gas pedal not pressed
    public static float maxspeed = 1.5f; // with gas pedal at full

    public static final int liftClearanceHeight = 1300;

    private static boolean timerstart=false;
    private static long ytimeout = 0;
    private static long btimeout = 0;
    private static long throwbackTime = 0;
    private static long throwdownTime = 0;
    private static long curtime;
    private static boolean inverted=false;
    private static long rbumptimer=0;
    private static long lbumptimer=0;
    private static long dpaddowntimer = 0;
    private static long dpaduptimer = 0;
    private static boolean dpadleftstart=false;
    private static boolean dpadrightstart=false;
    private static long dpadlefthold =0;
    private static long dpadrighthold = 0;
    private static long intakeArmMoving=0;
    private static long grabTimer = 0;
    private static boolean switching=false;
    private static boolean fromheld=false;
    private static boolean grabbing=false;

    private static int[] intakeArmPositions = {0,1500,1950};
    private static int intakeArmPos=0;

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
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        Gamepad1Update();
        Gamepad2Update();
        if(System.currentTimeMillis()-intakeArmMoving < 500) robot.intakeArm.setPower(0.5);
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

    public void Gamepad1Update() {
        //if the robot is not driving automatically, set motor power to the manual drive algorithm based on gamepad inputs
        if(!drive.getIsAutoMove())
        {
            float mult = gamepad1.right_trigger;
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
            drive.setMotorPower((rx)*mult, -(ry)*mult, (lx)*Math.abs(mult), -(ly)*mult, true);//if you change doFunction, make sure to also change it in RRBotAutoReader
        }
        else
        {
            drive.AutoMoveEndCheck();
        }
    }
    public void Gamepad2Update(){
        boolean ypressed = gamepad2.y;
        boolean apressed = gamepad2.a;
        boolean lbump = gamepad2.left_bumper;
        float ly = gamepad2.left_stick_y;
        boolean bpressed = gamepad2.b;
        boolean dpadup = gamepad2.dpad_up;
        boolean dpaddown = gamepad2.dpad_down;
        boolean dpadleft = gamepad2.dpad_left;
        boolean dpadright = gamepad2.dpad_right;
        if(gamepad2.left_trigger>0.5 && System.currentTimeMillis()-grabTimer > 500){
            robot.blockGrabber.setPosition(1-robot.blockGrabber.getPosition());
            grabTimer = System.currentTimeMillis();
        }

        liftMotorUpdate();

        if(dpadup && System.currentTimeMillis()-dpaduptimer > 500){
            if(intakeArmPos!=2) intakeArmPos++;
            robot.intakeArm.setTargetPosition(intakeArmPositions[intakeArmPos]);
            robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dpaduptimer = System.currentTimeMillis();
            intakeArmMoving = System.currentTimeMillis();
        }
        if(dpaddown && System.currentTimeMillis()-dpaddowntimer > 500){
            if(intakeArmPos!=0) intakeArmPos--;
            robot.intakeArm.setTargetPosition(intakeArmPositions[intakeArmPos]);
            robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dpaddowntimer = System.currentTimeMillis();
            intakeArmMoving = System.currentTimeMillis();
        }

        if(dpadleft){
            if(!dpadleftstart) {
                dpadlefthold=System.currentTimeMillis();
                dpadleftstart=true;
            }
            if(dpadleftstart && System.currentTimeMillis()-dpadlefthold>250){
                robot.endCapArm.setPosition(1-robot.endCapArm.getPosition());
                dpadleftstart=false;
            }
        }else{
            dpadleftstart=false;
        }
        if(dpadright){
            if(!dpadrightstart){
                dpadrighthold = System.currentTimeMillis();
                dpadrightstart=true;
            }
            if(dpadrightstart && System.currentTimeMillis()-dpadrighthold>250){
                flipLift();
                dpadrightstart=false;
            }
        }else{
            dpadrightstart=false;
        }

        if(lbump && System.currentTimeMillis()-lbumptimer > 500 && robot.liftMotor.getCurrentPosition()>liftClearanceHeight){
            robot.grabberArm.setPosition(1-robot.grabberArm.getPosition());
            lbumptimer = System.currentTimeMillis();
        }

        if(bpressed && System.currentTimeMillis()-btimeout > 250){
            robot.blockGrabber.setPosition(1-(robot.blockGrabber.getPosition()));
            btimeout = System.currentTimeMillis();
        }
        if(ypressed && System.currentTimeMillis()-ytimeout > 250){
            robot.trayPullerLeft.setPosition(1-robot.trayPullerLeft.getPosition());
            robot.trayPullerRight.setPosition(1-robot.trayPullerRight.getPosition());
            ytimeout = System.currentTimeMillis();
        }

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
    public void flipLift(){
        robot.liftMotor.setTargetPosition(liftClearanceHeight);
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
    public void liftMotorUpdate(){
        float ly = gamepad2.left_stick_y;
        if(ly<0){
            robot.liftMotor.setTargetPosition(2350);
        }else{
            robot.liftMotor.setTargetPosition(10);
        }
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(ly*0.3);
        telemetry.addData(">>",robot.liftMotor.getCurrentPosition());
        telemetry.update();
    }
}
