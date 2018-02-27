package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "TeleOp", name = "TeleOp")
public class OfficialTele extends LinearOpMode
{

    private SwerveDrive swerveDrive;

    public void runOpMode()
    {

        swerveDrive = new SwerveDrive(hardwareMap, telemetry, this);

        while(opModeIsActive())
        {
            double lx = gamepad1.right_stick_x;
            double ly = gamepad1.right_stick_y;
            double rx = gamepad1.left_stick_x;

            double strafeAngle = Math.atan2(lx, ly);
            double power = Math.sqrt((lx * lx) + (ly * ly));

            double turnAngle = 0;
            double turnPower = rx;

            if(rx < 0)
            {
                turnAngle = -90;
                turnPower *= -1;
            }
            else if(rx > 0)
            {
                turnAngle = 90;
            }
            else
            {
                turnAngle = 0;
            }


            turnAngle -= (power * 45);
            turnAngle *= turnPower;

            if(power == 0)
            {
                power = turnPower;
                turnAngle /= turnPower;
            }

            swerveDrive.setPower(power);
            swerveDrive.setTargets(strafeAngle, turnAngle);
            swerveDrive.updateSwerve();
        }

    }

}
