package org.firstinspires.ftc.teamcode.Test_Concepts;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

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
        updateGyro();

        if (!inProgress) {
            if (gamepad1.dpad_right) {
                desiredAngle = angles.firstAngle + 90;
                if (desiredAngle > 180) {
                    desiredAngle-= 360; // desiredAngle = desiredAngle - 360
                }
                inProgress = true;
                commenceTurn();

            }
            else if  (gamepad1.dpad_left) {
                desiredAngle = angles.firstAngle - 90;
                if (desiredAngle < -180) {
                    desiredAngle+= 360;
                }
                inProgress = true;
                commenceTurn();
            }

        }

        DcMotorEx motorEx = (DcMotorEx)topLeft;
        PIDCoefficients currentPID = getPID(motorEx);

        double p = currentPID.p;
        double i = currentPID.i;
        double d = currentPID.d;


        if (gamepad1.dpad_up) {
            setPID(p+0.01, i, d);
        }
        if (gamepad1.dpad_down) {
            setPID(p-0.01, i, d);
        }

        telemetry.addData("P Value", p);
        telemetry.addData("In Progress?", inProgress);
        telemetry.update();

    }


    public void commenceTurn(){
        double error = angles.firstAngle - desiredAngle;
        while (inProgress && Math.abs(error) > 2){
            updateGyro();
            error = angles.firstAngle - desiredAngle; // we may go the wrong way (e.g turn right when we want to turn left), but eh. We can fix that later.
            setMoveRobot(0,0, error); // in theory the PID will affect this directly.

            // Add some debug telemetry.
            telemetry.addData("Desired Angle", desiredAngle);
            telemetry.addData("Gyro Reading", angles.firstAngle);
            telemetry.addData("Error", error);
        }
        inProgress = false;
    }
}
