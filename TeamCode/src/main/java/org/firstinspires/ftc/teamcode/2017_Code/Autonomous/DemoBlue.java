package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Libraries.LibraryBaseAutonomous;

/**
 * Created by Ari on 12-12-17.
 */

@Autonomous(name="Demo Blue", group = "Demo")
public class DemoBlue extends LibraryBaseAutonomous {

    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode(){


        initBase();
        initConveyor();
        initGyro();
        initVuforia();

        telemetry.addData("Pray", "that it works.");
        telemetry.addData("Gotta", "love untested code.");
        telemetry.update();

        waitForStart();
        time.reset();

        while (opModeIsActive()){
            telemetry.addData("Time:",time.time());
            if (time.time() < 2){
                kicker.setPosition(0.9);
            }
            if (time.time() > 2 && time.time() < 3){
                moveDropKick(0.1);
            }
            else if (time.time() > 3 && time.time() < 4){
                dropKick.setPower(0);
                moveDropKick(0);
            }
            while (opModeIsActive() && time.time() > 4.1 && time.time() < 6.1){
                if (colour.red() < 13){
                    blueOnLeft = true;
                }
                else if (colour.red() > 13){
                    blueOnLeft = false;
                }
                else{
                    telemetry.addData("Jewel Location", "Unknown");
                }
            }

            if(blueOnLeft){
                if(time.time()>6 && time.time()<7){
                    setMoveRobot(0.25,0,0);
                    telemetry.addData("I like", "spagety");
/*
                }else if(time.time()>6 && time.time()<7){
                    setMoveRobot(-0.25,0,0);
*/
                }else if(time.time()>7 && time.time() < 7.4){

                    setMoveRobot(0,0,0);
                    kicker.setPosition(0.1);

                }

            }else{
                if(time.time()>6 && time.time()<7){
                    setMoveRobot(-0.25,0,0);
                    telemetry.addData("I like", "nudls");
/*
                }else if(time.time()>6 && time.time()<7){
                    setMoveRobot(0.25,0,0);
*/
                }else if(time.time()>7 && time.time()<7.4) {
                    setMoveRobot(0, 0, 0);
                    kicker.setPosition(0.1);


                }
            }
            telemetry.addData("Color", colour.red());

            telemetry.update();
        }
    }
}
