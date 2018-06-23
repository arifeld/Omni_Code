package org.firstinspires.ftc.teamcode.Test_Concepts;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "Turning PID", group = "Testing")
public class Precise_Turning_PID extends Temp_Library {

    private double kP = 0.69;
    private Orientation angles;
    private boolean inProgress = false;
    private float desiredAngle = 0;
    private DcMotorEx motorEx = (DcMotorEx)topLeft;

    @Override
    public void init(){
        telemetry.addData("Program", "P-Turn Test");
        initBase();
        initGyro();
        telemetry.update();
        PIDCoefficients currentPID = getPID(motorEx);
        setPID(kP, currentPID.i, currentPID.d);
    }

    public void start(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Initial Angle", angles);
    }

    public void loop(){
        telemetry.addData("Current K value", kP);
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

        PIDCoefficients currentPID = getPID(motorEx);

        double kP = currentPID.p;
        double i = currentPID.i;
        double d = currentPID.d;


        if (gamepad1.dpad_up) {
            kP += 0.01;
            setPID(kP, i, d);
        }
        if (gamepad1.dpad_down) {
            kP -= 0.01;
            setPID(kP, i, d);
        }

        telemetry.addData("P Value", kP);
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
