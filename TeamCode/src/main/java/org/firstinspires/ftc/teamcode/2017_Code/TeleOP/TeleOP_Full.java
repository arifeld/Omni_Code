package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.LibraryBaseTeleOP;

/**
 * Created by Ari on 02-11-17.
 * Edited by Harrison
 */



@TeleOp(name = "TeleOP_Full", group = "Comp")
public class TeleOP_Full extends LibraryBaseTeleOP {

    private double gyroYaw    = 0;
    private double dAxial     = 0;
    private double dLateral   = 0;
    private double dYaw       = 0;

    private double intakePower = 0.025;






    @Override
    public void init(){
        initBase(); // From library, inits, sets direction, and resets all base motor.
        initConveyor();
        initGyro();
        telemetry.addData("STATUS: ", "Initialised.");
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


        //Translations
        updateGyro();

        if (gamepad1.a){
            dAxial   = -gamepad1.left_stick_y;
            dLateral =  gamepad1.left_stick_x;
            dYaw     = -gamepad1.right_stick_x/3;
        }
        else{
            dAxial   = -gamepad1.left_stick_y/3;
            dLateral =  gamepad1.left_stick_x/3;
            dYaw     = -gamepad1.right_stick_x/3;
        }

        setMoveRobot(dAxial, dLateral, dYaw);

        gyroYaw  = angles.firstAngle;

        // Add some telemetry.
        telemetry.addData("Axial: ", dAxial);
        telemetry.addData("Lateral: ", dLateral);
        telemetry.addData("Yaw: ", dYaw);
        telemetry.addData("Gyro Yaw: ", gyroYaw);


        //Turn the conveyor
        moveConveyor(-gamepad2.right_stick_y/20);

        //Drop intake
        if(gamepad2.right_bumper){
            moveDropKick(-0.1);
        }else if(gamepad2.left_bumper){
            moveDropKick(0.1);
        }else{
            moveDropKick(0);
        }
        //Turn intake
        if (gamepad2.left_trigger > 0.2 || gamepad1.left_trigger > 0.2){ // May need to change this.
            moveIntake(intakePower);
        }
        else if (gamepad2.right_trigger > 0.2 || gamepad1.right_trigger > 0.2){
            moveIntake(-intakePower);
        }else{
            moveIntake(0);
        }


        if(gamepad2.y){
            kicker.setPosition(0);
        }else if (gamepad2.b){
            kicker.setPosition(0.25);
        }else if(gamepad2.a){
            kicker.setPosition(0.45);
        }




        telemetry.update();
    }

    @Override
    public void stop(){


    }

}