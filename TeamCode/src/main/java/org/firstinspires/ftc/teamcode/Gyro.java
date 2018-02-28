package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro
{

    //We need a good deal of objects in this class
    private BNO055IMU gyro;
    private double cumulativeAngle;
    private Orientation lastAngles;
    private LinearOpMode op;
    private Thread getAngles;
    private Telemetry telemetry;

    //constructor
    public Gyro(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode op)
    {
        //setting some object states
        this.op = op;
        cumulativeAngle = 0;
        this.telemetry = telemetry;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        //IMU should be named "gyro" as an AdaFruit IMU attached to an I2C port
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");

        gyro.initialize(parameters); //Initialize gyro

        telemetry.addData("Gyro Calibrating... don't move", null); //telemetry stuff
        telemetry.update();

        //wait for the gyro to finish calibrating before starting anything
        while (!gyro.isGyroCalibrated()) {}
        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.clear();
        telemetry.addData("Gyro Calibrated.", null); //yay
        telemetry.update();
    }

    //Because of the way the gyro works (returns value between -180 and 180, if you go to -181 it goes to 179), it can be critical to read constantly.  If we don't, it could glitch and go to value 361 instead of 1.  To assure that doesn't happen, we read constantly off a thread
    public void startGyro()
    {
        getAngles = new Thread(calculateAngles);
        getAngles.start();
    }

    //To stop the gyro, we simply need to interrupt the thread
    public void stopGyro()
    {
        getAngles.interrupt();
    }

    //If for any reason we want to reset the angle of the gyro
    public void resetAngle()
    {
        cumulativeAngle = 0;
    }

    //Getting the angle from the gyro.  This method could be renamed to updateAngleValue, because that is really what it does
    public void getAngle()
    {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double degree = angles.firstAngle - lastAngles.firstAngle;

        //adding or subracting 360 depending on whether we went to -181 or not.  The reason we need to read constantly
        if(degree < -180)
        {
            degree+=360;
        }
        else if(degree > 180)
        {
            degree -= 360;
        }
        cumulativeAngle += degree; //this is the key angle, it is the absolute angle of the gyro in relation to its starting position.  If the gyro has done 2 full rotations, this will be 720 instead of 0.

        lastAngles = angles; //Now lastAngles are the angles we just used
    }

    //getter
    public double getCumulativeAngle()
    {
        return cumulativeAngle;
    }

    //Runnable for the thread
    private Runnable calculateAngles = new Runnable()
    {
      public void run()
      {
          while(op.opModeIsActive() && !Thread.currentThread().isInterrupted()) //Reason thread kills when its interrupted
          {
              getAngle();
              telemetry.clear();
              telemetry.addData("Intrinsic Angle:  ", cumulativeAngle);
              telemetry.update();
          }
      }
    };

}
