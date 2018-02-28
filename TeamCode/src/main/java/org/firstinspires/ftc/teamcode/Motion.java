package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Motion
{

    //Swerve drive requires a lot of dcmotors...
    private DcMotor bl;
    private DcMotor br;
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bla;
    private DcMotor bra;
    private DcMotor fla;
    private DcMotor fra;
    private double countsPerDegree;
    private double power;
    private double frontTarget;
    private double backTarget;
    private double frontPosition;
    private double backPosition;
    private LinearOpMode op;
    private Gyro gyro;

    //constructor
    public Motion(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode op)
    {
        //setting variables and objects
        countsPerDegree = 1120 / 360;
        power = 0;
        frontTarget = 0;
        backTarget = 0;
        frontPosition = 0;
        backPosition = 0;
        this.op = op;
        gyro = new Gyro(hardwareMap, telemetry, op);

        //setting motors
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bla = hardwareMap.get(DcMotor.class, "bla");
        bra = hardwareMap.get(DcMotor.class, "bra");
        fla = hardwareMap.get(DcMotor.class, "fla");
        fra = hardwareMap.get(DcMotor.class, "fra");

        //setting motor directions
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bla.setDirection(DcMotor.Direction.FORWARD);
        bra.setDirection(DcMotor.Direction.REVERSE);
        fla.setDirection(DcMotor.Direction.FORWARD);
        fra.setDirection(DcMotor.Direction.REVERSE);

        //resetting all encoders
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bla.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fla.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //setting motor modes, run_using_encoder is for the gyro-encoder mixed controller while runToPosition doesn't require anything but default encoders
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bla.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fla.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fra.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Making the motors brake when they aren't being powered
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bla.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fla.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //starting the gyro thread
        gyro.startGyro();
    }

    //updates all of the target positions and powers
    public void updateSwerve()
    {
        fla.setPower(1);
        fra.setPower(1);
        bla.setPower(1);
        bra.setPower(1);

        fla.setTargetPosition((int) (gyro.getCumulativeAngle() * countsPerDegree + frontTarget));
        fra.setTargetPosition((int) (gyro.getCumulativeAngle() * countsPerDegree + frontTarget));
        bla.setTargetPosition((int) (gyro.getCumulativeAngle() * countsPerDegree + backTarget));
        bra.setTargetPosition((int) (gyro.getCumulativeAngle() * countsPerDegree + backTarget));

        bl.setPower(power);
        br.setPower(power);
        fl.setPower(power);
        fr.setPower(power);
    }

    //setting the targets according to new variables
    public void setTargets(double strafeAngle, double turnAngle)
    {
        //Set the targets in terms of degrees on the coordinate plane, front and back differ because of turning
        frontTarget = strafeAngle + turnAngle;
        backTarget = strafeAngle - turnAngle;

        //Set the front and back position according to what they are.  This is faster than constantly calling getCurrentPosition(), and ensures accurate math
        frontPosition = fla.getCurrentPosition();
        backPosition = bla.getCurrentPosition();

        //Subtracting or adding 360 to the currentPosition variables to get their coterminal angles that are less than 360 because the targets are that way
        double coTerminalFront = frontPosition;
        double coTerminalRear = backPosition;
        if(coTerminalFront < 0)
        {
            while(coTerminalFront < 360)
            {
                coTerminalFront += 360;
            }
        }
        else
        {
            while(coTerminalFront > 360)
            {
                coTerminalFront -= 360;
            }
        }
        if(coTerminalRear < 0)
        {
            while(coTerminalRear < 360)
            {
                coTerminalRear += 360;
            }
        }
        else
        {
            while(coTerminalRear > 360)
            {
                coTerminalRear -= 360;
            }
        }

        //Getting the distance required to travel.  If distance > 180 (the max), then it will be more efficient to calculate a way to cross 360 or 0 to get to the angle.  370 and 10 are the same angle, and 350 is closer to 370 than to 10
        double DTTFront = frontTarget - coTerminalFront;
        double DTTBack = backTarget - coTerminalRear;

        if(DTTFront > 180)
        {
            DTTFront = 0 - coTerminalFront - 360 + frontTarget;
        }
        else if(DTTFront < -180)
        {
            DTTFront = 0 + frontTarget + 360 - coTerminalFront;
        }

        if(DTTBack > 180)
        {
            DTTBack = 0 - coTerminalRear - 360 + backTarget;
        }
        else if(DTTFront < -180)
        {
            DTTBack = 0 + backTarget + 360 - coTerminalRear;
        }

        //The DTTFront and DTTBack variables are the calculated distance needed to travel to reach the target position, so to get the targets we simply need to add them to the current position
        frontTarget = (frontPosition + DTTFront) * countsPerDegree;
        backTarget = (backPosition + DTTBack) * countsPerDegree;


    }

    //Power setter, as you can see it is way more complicated than the angle setter
    public void setPower(double power)
    {
        this.power = power;
    }

    //Method used to turn, it uses a combination of the gyro, basic algebra, and the encoder power PID
    public void turn(double target, double speed)
    {
        //set variables that are required
        double start = gyro.getCumulativeAngle();
        double error = target + start - gyro.getCumulativeAngle();
        double maxSpeed = 0;

        //setting the targets of the motor angle things so that they will be facing opposite directions, then set their power so
        //they move to the correct position (thanks built in PIDs)
        setTargets(0, 90);
        fra.setPower(1);
        fla.setPower(1);
        bra.setPower(1);
        bla.setPower(1);
        while(fra.isBusy() || fla.isBusy() || bra.isBusy() || bla.isBusy()) //Waiting for them to finish moving
        {

        }

        //Making them brake so that they don't move out of position
        fra.setPower(0);
        fla.setPower(0);
        bra.setPower(0);
        bla.setPower(0);

        //Main while loop, does this until it reaches the target
        double lastSpeed = 0; //lastSpeed for gradual acceleration
        while(error != target)
        {
            boolean notDecelerating = false;
            maxSpeed = error / 30; //Division ensures gradual and linear deceleration
            if(maxSpeed > speed) //max speed
            {
                maxSpeed = speed;
                notDecelerating = true;
            }
            else if(maxSpeed < -speed) //max speed
            {
                maxSpeed = -speed;
                notDecelerating = true;
            }
            else if(maxSpeed < 0.2 && maxSpeed >= 0) //minimum speed
            {
                maxSpeed = .2;
            }
            else if(maxSpeed > -.2 && maxSpeed <= 0) //minimum speed
            {
                maxSpeed = -.2;
            }

            //gradual acceleration if we are not decelerating
            if(lastSpeed < speed && notDecelerating)
            {
                maxSpeed = lastSpeed + 0.05;
            }

            //setting the power of the motors.  The power is the same since they face opposite directions
            fra.setPower(maxSpeed);
            fla.setPower(maxSpeed);
            bra.setPower(maxSpeed);
            bla.setPower(maxSpeed);

            error = target + start - gyro.getCumulativeAngle(); //Recalculating error
        }
        //Braking the motors so we don't spin forever
        fra.setPower(0);
        fla.setPower(0);
        bra.setPower(0);
        bla.setPower(0);
    }

}
