package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Misc.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.Misc.MathFunctions.lineCircleIntersection;
import static org.firstinspires.ftc.teamcode.Misc.RRBotHardware.*;

/**
 * Controls the robot's mecanum drive base
 * @author Andrew Hollabaugh
 * @since 2017-10-08
 */
public class RRBotMecanumDrive
{
    RRBotHardware robot;

    private ElapsedTime autoMoveTime = new ElapsedTime();

    private boolean isAutoMove = false;
    private double autoTime;

    /**
     * Constructor gets hardware object from teleop class
     * @param robot contains the hardware elements of the robot
     */
    public RRBotMecanumDrive(RRBotHardware robot)
    {
        this.robot = robot;
    }

    /**
     * Calculates the velocity values for the drive motors in a mecanum configuration.
     * @param leftX X position of left joystick
     * @param leftY Y position of left joystick
     * @param rightX X position of right joystick
     * @param rightY Y position of right joystick
     * @param doFunction whether a function should be used on the input values
     * @return velocities - array of motor velocities
     */
    public double[] calcVelocities(double leftX, double leftY, double rightX, double rightY, boolean doFunction)
    {
        double moveX = rightX;
        double moveY1 = leftY;
        double turn = leftX;
        double moveY2 = rightY;

        //remap input values using a function
        if(doFunction)
        {
            moveX = inputFunction(moveX);
            moveY1 = inputFunction(moveY1);
            turn = inputFunction(turn);
            moveY2 = inputFunction(moveY2);
        }

        double v1 = moveY1 + moveX + turn + moveY2;
        double v2 = moveY1 - moveX - turn + moveY2;
        double v3 = moveY1 + moveX - turn + moveY2;
        double v4 = moveY1 - moveX + turn + moveY2;

        double max = Math.abs(v1);
        if(Math.abs(v2) > max)
            max = Math.abs(v2);
        if(Math.abs(v3) > max)
            max = Math.abs(v3);
        if(Math.abs(v4) > max)
            max = Math.abs(v4);
        if(max > 1)
        {
            v1 /= max;
            v2 /= max;
            v3 /= max;
            v4 /= max;
        }

        double[] velocities = {v1, v2, v3, v4};
        return velocities;
    }

    /**
     * Sets the motor power for manual drive. The parameters are sent to calcVelocities.
     * @param leftX X position of left joystick
     * @param leftY Y position of left joystick
     * @param rightX X position of right joystick
     * @param rightY Y position of right joystick
     * @param doFunction whether a function should be used on the input values
     */
    public void setMotorPower(double leftX, double leftY, double rightX, double rightY, boolean doFunction)
    {
        //calculate the velocities
        double[] velocities = calcVelocities(leftX, leftY, rightX, rightY, doFunction);

        //set the motor power
        robot.frontLeftMotor.setPower(velocities[0]);
        robot.frontRightMotor.setPower(velocities[1]);
        robot.rearLeftMotor.setPower(velocities[2]);
        robot.rearRightMotor.setPower(velocities[3]);
    }

    /**
     * Function to be executed on the joystick input values. Not currently used
     * @param input joystick value
     * @return value - input value scaled by function
     */
    public double inputFunction(double input)
    {
        //double value = input;

        //f(x) = x^3
        //value = value * value * value;
        //return value;

        /*if(value > 0)
        {
            Range.clip(value, driveDeadBand, 1);
        }
        else if(value < 0)
        {
            Range.clip(value, -1, -driveDeadBand);
        }*/
        /*if(Math.abs(value - 0) < 0.01)
        {
            value = 0;
        }*/

        /*if(input >= 0)
        {
            value = Math.sqrt(input);
        }
        else
        {
            value = -Math.sqrt(-input);
        }
        return value;*/

        //f(x) = x^2
        double value = input * input;

        if(input < 0)
        {
            return value * -1;
        }
        else
        {
            return value;
        }
    }

    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        CurvePoint followMe = getFollowPointPath(allPoints,new Point(worldXPosition,worldYPosition),allPoints.get(0).followDistance);
        goToPosition(followMe.x,followMe.y,followMe.moveSpeed,followAngle,followMe.turnSpeed);
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        for(int i=0;i<pathPoints.size()-1;i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);
            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(),endLine.toPoint());
            double closestAngle = 1000000000;
            for(Point thisIntersection : intersections){
                double angle = Math.atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition);
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - worldAngle_rad));

                if(deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public void ApplyMovement() {

        double tl_power_raw = MovementVars.movement_y-MovementVars.movement_turn+MovementVars.movement_x*1.5;
        double bl_power_raw = MovementVars.movement_y-MovementVars.movement_turn- MovementVars.movement_x*1.5;
        double br_power_raw = -MovementVars.movement_y-MovementVars.movement_turn-MovementVars.movement_x*1.5;
        double tr_power_raw = -MovementVars.movement_y-MovementVars.movement_turn+MovementVars.movement_x*1.5;

        //find the maximum of the powers
        double maxRawPower = Math.abs(tl_power_raw);
        if(Math.abs(bl_power_raw) > maxRawPower){ maxRawPower = Math.abs(bl_power_raw);}
        if(Math.abs(br_power_raw) > maxRawPower){ maxRawPower = Math.abs(br_power_raw);}
        if(Math.abs(tr_power_raw) > maxRawPower){ maxRawPower = Math.abs(tr_power_raw);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        tl_power_raw *= scaleDownAmount;
        bl_power_raw *= scaleDownAmount;
        br_power_raw *= scaleDownAmount;
        tr_power_raw *= scaleDownAmount;

        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
        robot.frontLeftMotor.setPower(tl_power_raw);
        robot.rearLeftMotor.setPower(bl_power_raw);
        robot.rearRightMotor.setPower(br_power_raw);
        robot.frontRightMotor.setPower(tr_power_raw);
    }

    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed){
        double distanceToTarget = Math.hypot(x-worldXPosition, y-worldYPosition);
        double absoluteAngleToTarget = Math.atan2(y-worldYPosition,x-worldXPosition);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        MovementVars.movement_x = movementXPower * movementSpeed;
        MovementVars.movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        MovementVars.movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1) * turnSpeed;

        if(distanceToTarget < 10){
            MovementVars.movement_turn = 0;
        }
    }

    /**
     * Automatically moves the robot based on a set speed and time. Meant to be used in teleop.
     * @param speed speed of movement
     * @param time how long the movement should take
     */
    public void AutoMove(double speed, double time)
    {
        isAutoMove = true;
        autoTime = time;

        autoMoveTime.reset();

        robot.rearRightMotor.setPower(speed);
        robot.rearLeftMotor.setPower(speed);
        robot.frontRightMotor.setPower(speed);
        robot.frontLeftMotor.setPower(speed);
    }

    /**
     * Checks if the movement is done by comparing the time elapsed and the time the movement is set to take.
     */
    public void AutoMoveEndCheck()
    {
        if(autoMoveTime.milliseconds() >= autoTime)
        {
            isAutoMove = false;
        }
    }

    /**
     * Returns whether an auto move is currently occuring
     * @return isAutoMove
     */
    public boolean getIsAutoMove()
    {
        return isAutoMove;
    }
}
