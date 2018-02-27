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

    private BNO055IMU gyro;
    private double cumulativeAngle;
    private Orientation lastAngles;
    private LinearOpMode op;
    private Thread getAngles;
    private Telemetry telemetry;

    public Gyro(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode op)
    {
        this.op = op;
        cumulativeAngle = 0;
        this.telemetry = telemetry;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "gyro".
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");

        gyro.initialize(parameters);

        telemetry.addData("Gyro Calibrating... don't move", null);
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!gyro.isGyroCalibrated()) {}
        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.clear();
        telemetry.addData("Gyro Calibrated.", null);
        telemetry.update();
    }

    //The gyro can glitch if you don't constantly read.

    public void startGyro()
    {
        getAngles = new Thread(calculateAngles);
        getAngles.start();
    }

    public void stopGyro()
    {
        getAngles.interrupt();
    }

    public void resetAngle()
    {
        cumulativeAngle = 0;
    }

    public void getAngle()
    {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double degree = angles.firstAngle - lastAngles.firstAngle;

        if(degree < -180)
        {
            degree+=360;
        }
        else if(degree > 180)
        {
            degree -= 360;
        }
        cumulativeAngle += degree;

        lastAngles = angles;
    }

    public double getCumulativeAngle()
    {
        return cumulativeAngle;
    }

    private Runnable calculateAngles = new Runnable()
    {
      public void run()
      {
          while(op.opModeIsActive() && !Thread.currentThread().isInterrupted())
          {
              getAngle();
              telemetry.clear();
              telemetry.addData("Intrinsic Angle:  ", cumulativeAngle);
              telemetry.update();
          }
      }
    };

}
