package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.LibraryBaseTeleOP;

/**
 * Created by Ari on 10-10-17.
 */

@TeleOp(name="Conveyor Test", group="Comp")
public class Conveyor_Test extends LibraryBaseTeleOP {


    @Override
    public void init(){
        initConveyor();
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        moveConveyor(-gamepad2.right_stick_y);
        telemetry.addData("Power: ", -gamepad2.right_stick_y);
        telemetry.update();
    }

    @Override
    public void stop(){


    }
}

