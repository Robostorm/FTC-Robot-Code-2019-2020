package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Misc.RRBotHardware;
import org.firstinspires.ftc.teamcode.Misc.RRBotMecanumDrive;

import java.io.BufferedWriter;
import java.io.IOException;

@TeleOp(name="RRBotRecorder")
public class RRBotRecorder extends OpMode{
    // Declare OpMode members.
    public static final String RECORD_FILE = "teleop_rec.txt";

    private ElapsedTime runtime = new ElapsedTime();
    private static boolean startedRecording=false;
    private static long curtime=-1;

    BufferedWriter writer;
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

        /*try{

            telemetry.addData(">>","Writer initialized.");
        }catch(IOException e){
            telemetry.addData(">>",e.getMessage());
            //telemetry.addData(">>", );
        }*/
    }

    @Override
    public void loop() {
        DriveUpdate();
    }

    @Override
    public void stop() {
        try{
            writer.close();
        }catch(IOException e) {e.printStackTrace();}
    }

    public void DriveUpdate()
    {
        //if the robot is not driving automatically, set motor power to the manual drive algorithm based on gamepad inputs
        if(!drive.getIsAutoMove())//CHECK TO SEE HOW MANY MILLIS BETWEEN EACH RUN OF THIS, SEE IF YOU NEED TO STANDARDIZE
        {
            //if(curtime!=-1 && System.currentTimeMillis()-curtime >=15){}
            float leftx = gamepad1.left_stick_x;
            float lefty = gamepad1.left_stick_y;
            float rightx = -gamepad1.right_stick_x;
            float righty = gamepad1.right_stick_y;
            boolean a = gamepad1.a;
            float gas = gamepad1.right_trigger;
            boolean x = gamepad1.x;
            if(!startedRecording && (leftx != 0 || lefty != 0 || rightx!= 0 || righty !=0) || a==true || gas!=0 || x==true){
                telemetry.addData(">>","Started recording.");
                telemetry.update();
                startedRecording = true;
            }else if(startedRecording){
                String message = leftx+","+lefty+","+rightx+","+righty+","+a+","+gas+","+x;
                try{
                    writer.append(message+"\n");
                }catch (IOException e) {e.printStackTrace();}
            }
        }
        else
        {
            drive.AutoMoveEndCheck();
        }
    }
}
