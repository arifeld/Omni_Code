package org.firstinspires.ftc.teamcode.Test_Concepts;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Precise Turning Test", group = "Testing")
public class Precise_Turning extends Temp_Library {

    private double k_value = 0.008;
    private Orientation angles;
    private boolean inProgress = false;
    private float desiredAngle = 0;

    @Override
    public void init(){
        telemetry.addData("Program", "P-Turn Test");
        initBase();
        initGyro();
        telemetry.update();
    }

    public void start(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Initial Angle", angles);
    }

    public void loop(){
        telemetry.addData("Current K value", k_value);

        if (!inProgress) {
            if (gamepad1.dpad_right) {
                desiredAngle = angles.firstAngle + 90;
                if (desiredAngle > 180) {
                    desiredAngle-= 360; // desiredAngle = desiredAngle - 360
                }
                actonTurn();
                inProgress = true;
            }
            else if  (gamepad1.dpad_left) {
                desiredAngle = angles.firstAngle - 90;
                if (desiredAngle < -180) {
                    desiredAngle+= 360;
                }
                actonTurn();
                inProgress = true;
            }


        }
        if (gamepad1.dpad_up) {
            k_value+=0.01;
        }
        if (gamepad1.dpad_down) {
            k_value-=-0.01;
        }

    }

    public void actonTurn() {
        while (inProgress) {
            if (desiredAngle > 0) {
                double turnPower = desiredAngle - angles.firstAngle;
                double yaw = k_value * turnPower; // why does lambda not exist? I guess this does the same though.
                setMoveRobot(0, 0, yaw);

                if (-3 < turnPower && turnPower < 3) {
                    setMoveRobot(0, 0, 0);
                    inProgress = false;
                }
            } else if (desiredAngle < 0) {
                double turnPower = desiredAngle + angles.firstAngle; // THIS LINE IS PROBABLY WRONG. BUT IM TIRED SO I'LL FIX IT LATER.
                double yaw = k_value * turnPower;
                setMoveRobot(0, 0, yaw);

                // Haven't actually programmed in a stop method..

                if (-3 < turnPower && turnPower < 3) {
                    setMoveRobot(0, 0, 0);
                    inProgress = false;
                }
            }

        }
    }
}
