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
    private double distanceToTravelFront;
    private double distanceToTravelBack;
    private double countsPerDegree;
    private double power;
    private double strafeAngle;
    private LinearOpMode op;
    private Gyro gyro;

    public SwerveDrive(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode op)
    {
        distanceToTravelBack = 0;
        distanceToTravelFront = 0;
        countsPerDegree = 1120 / 360;
        strafeAngle = 0;
        power = 0;
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
    }

    public void runSwerve()
    {
        fla.setPower(1);
        fra.setPower(1);
        bla.setPower(1);
        bra.setPower(1);
        gyro.startGyro();

        while(op.opModeIsActive())
        {
            fla.setTargetPosition((int) (strafeAngle + distanceToTravelFront + gyro.getCumulativeAngle() * countsPerDegree + fla.getCurrentPosition()));
            fra.setTargetPosition((int) (strafeAngle + distanceToTravelFront + gyro.getCumulativeAngle() * countsPerDegree + fra.getCurrentPosition()));
            bla.setTargetPosition((int) (strafeAngle + distanceToTravelBack + gyro.getCumulativeAngle() * countsPerDegree + bla.getCurrentPosition()));
            bra.setTargetPosition((int) (strafeAngle + distanceToTravelBack + gyro.getCumulativeAngle() * countsPerDegree + fra.getCurrentPosition()));

            distanceToTravelBack = 0;
            distanceToTravelFront = 0;

            bl.setPower(power);
            br.setPower(power);
            fl.setPower(power);
            fr.setPower(power);
            op.idle();
        }
    }

    public void setStrafe(double strafeAngle)
    {
        this.strafeAngle = strafeAngle * countsPerDegree;
    }

    public void setTurn(double angle)
    {
        distanceToTravelFront = angle * countsPerDegree;
        distanceToTravelBack = -angle * countsPerDegree;
    }

    public void setPower(double power)
    {
        this.power = power;
    }

}
