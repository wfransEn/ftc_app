package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwerveDrive
{

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

    public SwerveDrive(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode op)
    {
        countsPerDegree = 1120 / 360;
        power = 0;
        frontTarget = 0;
        backTarget = 0;
        frontPosition = 0;
        backPosition = 0;
        this.op = op;
        gyro = new Gyro(hardwareMap, telemetry, op);

        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bla = hardwareMap.get(DcMotor.class, "bla");
        bra = hardwareMap.get(DcMotor.class, "bra");
        fla = hardwareMap.get(DcMotor.class, "fla");
        fra = hardwareMap.get(DcMotor.class, "fra");

        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bla.setDirection(DcMotor.Direction.FORWARD);
        bra.setDirection(DcMotor.Direction.REVERSE);
        fla.setDirection(DcMotor.Direction.FORWARD);
        fra.setDirection(DcMotor.Direction.REVERSE);

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bla.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fla.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bla.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bra.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fla.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fra.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bla.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fla.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gyro.startGyro();
    }

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

    public void setTargets(double strafeAngle, double turnAngle)
    {
        //Set the angles in terms of degrees on the coordinate plane.
        frontTarget = strafeAngle + turnAngle;
        backTarget = strafeAngle - turnAngle;

        frontPosition = fla.getCurrentPosition();
        backPosition = bla.getCurrentPosition();

        //Dividing front and back positions until they are less than 360 because they can loop around 360, and the targets
        //are in terms of a single rotation coordinate plane.
        double coTerminalFront = frontPosition;
        double coTerminalRear = backPosition;
        if(coTerminalFront < 0)
        {
            while(coTerminalFront < -360)
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
            while(coTerminalRear < -360)
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

        //Getting the distance to travel to check which direction we need to travel, these are guesses and if they are greater than
        //180 (the max), then we will do the other way.
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

        //The DTTFront and DTTBack variables are the calculated distance needed to travel to reach the target position, so to get the targets we simply
        //need to add them to the current position.
        frontTarget = frontPosition + DTTFront;
        backTarget = backPosition + DTTBack;


    }

    public void setPower(double power)
    {
        this.power = power;
    }

}
