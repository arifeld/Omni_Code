package org.firstinspires.ftc.teamcode.Test_Concepts;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Ari on 21-06-18.
 */
@TeleOp(name="Driving Calculations")
public class Driving_Calculations extends Temp_Library {
    private Orientation angles;
    private boolean currentlyRunning = false;
    private ElapsedTime time = new ElapsedTime();

    public void init(){
        initBase();
        initGyro();
        telemetry.addData("STATUS", "Hopefully initialised.");
    }

    public void init_loop(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


    }
    public void start(){

    }

    public void loop(){
        if (gamepad1.a && !currentlyRunning) {
            currentlyRunning = true; // Tell us to go forward.
        }

        if (currentlyRunning){
            time.reset(); // Reset our timer.
            while (currentlyRunning && (time.time() <= 1) ){ // Whilst (hopefully) the timer is less than / equal to 1.
                setMoveRobot(1,0,0); // Go forward.
                telemetry.addData("Time", time.time());
                telemetry.addData("topLeftE", topLeft.getCurrentPosition());
                telemetry.addData("topRightE", topRight.getCurrentPosition());
                telemetry.addData("backLeftE", backLeft.getCurrentPosition());
                telemetry.addData("backRightE", backRight.getCurrentPosition());
                telemetry.update();
            }
            setMoveRobot(0,0,0);
            currentlyRunning = false;
        }


        if (gamepad1.left_bumper){
            PIDCoefficients pid = new PIDCoefficients(1,1,1);
        }
    }


}
