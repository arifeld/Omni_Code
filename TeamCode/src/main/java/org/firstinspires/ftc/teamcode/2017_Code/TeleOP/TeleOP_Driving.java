package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.LibraryBaseTeleOP;

/**
 * Created by Ari on 10-10-17.
 */

@TeleOp(name="10496 TeleOP_Driving", group="Comp")
public class TeleOP_Driving extends LibraryBaseTeleOP {

    private double gyroYaw    = 0;
    private double dAxial     = 0;
    private double dLateral   = 0;
    private double dYaw       = 0;


    @Override
    public void init(){
        initBase(); // From library, inits, sets direction, and resets all base motor.
        initGyro();
        telemetry.update();
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){

        updateGyro();
        dAxial   = -gamepad1.left_stick_y;
        dLateral =  gamepad1.left_stick_x;
        dYaw     = -gamepad1.right_stick_x;
        gyroYaw  = angles.firstAngle;

        // Add some telemetry.
        telemetry.addData("Axial: ", dAxial);
        telemetry.addData("Lateral: ", dLateral);
        telemetry.addData("Yaw: ", dYaw);
        telemetry.addData("Gyro Yaw: ", gyroYaw);
        telemetry.addData("Mode:", topRight.getMode());

        setMoveRobot(dAxial, dLateral, dYaw);

        telemetry.update();
    }

    @Override
    public void stop(){


    }
}

