package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "TeleOp", name = "TeleOp")
public class Tele extends LinearOpMode
{

    private Motion motion;

    public void runOpMode()
    {

        motion = new Motion(hardwareMap, telemetry, this);

        while(opModeIsActive())
        {
            //Getting joystick inputs to drive the swerve drive
            double lx = gamepad1.right_stick_x;
            double ly = gamepad1.right_stick_y;
            double rx = gamepad1.left_stick_x;

            //Setting the strafe angle and power according to the left stick axes
            double strafeAngle = Math.atan2(lx, ly); //Free trig, thanks oracle (too bad we wasted our time making this function only to find out its built in)
            double power = Math.sqrt((lx * lx) + (ly * ly)); //Pythagorean Theorem

            double turnAngle = 0; //Yes, calculating turn direction
            double turnPower = rx; //Turn power.  Useful in deciding how much weight the turn should have against strafing speed

            if(rx < 0) //If left, then turnAngle should be -90.  Turn power needs to be positive
            {
                turnAngle = -90;
                turnPower *= -1;
            }
            else if(rx > 0) //If it's right, set it positive.
            {
                turnAngle = 90;
            }
            else
            {
                turnAngle = 0;
            }


            //Diminishing the turnAngle because the faster we turn, the slower we strafe.  Depending on how far control sticks are pressed, the turn can be anything from 0 (none) to 90 (pivoting, no strafing)
            turnAngle -= (power * 45);
            turnAngle *= turnPower;

            //If we aren't strafing, we don't want turn angle to be diminished at all because we would strafe a little if it were
            if(power == 0)
            {
                turnAngle /= turnPower; //Setting it back to 90 because if strafe power is 0, all we did was divide it by power
            }

            power += turnPower; //We want power to increase if we are turning so we can still drive the same speed
            if(power > 1)
            {
                power = 1; //Limiting power to 1
            }

            //Calling upon the motion methods to update the variables and set targets and powers to the new positions, or the old ones
            motion.setPower(power);
            motion.setTargets(strafeAngle, turnAngle);
            motion.updateSwerve();
        }

    }

}
