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

    public void DriveUpdate()
    {
        //if the robot is not driving automatically, set motor power to the manual drive algorithm based on gamepad inputs
        if(!drive.getIsAutoMove())
        {
            //drive.setMotorPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.right_stick_y, true);
            drive.setMotorPower(gamepad1.left_stick_x/2, -gamepad1.left_stick_y/2, -gamepad1.right_stick_x/2, -gamepad1.right_stick_y/2, true);
        }
        else
        {
            drive.AutoMoveEndCheck();
        }
    }
}
