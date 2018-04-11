package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Libraries.LibraryBaseAutonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Harrison on 22/11/2017.
 */
@Autonomous(name="BlueA")
public class Autonomous_BlueA extends LibraryBaseAutonomous {

    public double gyroFirst;
    ElapsedTime time = new ElapsedTime();


    @Override
    public void runOpMode() {
        initBase();
        initConveyor();
        initGyro();
        initVuforia();

        waitForStart();
        activateTracking();

        while (opModeIsActive()) {
            // Add some telemetry.
            telemetry.addData("Current Time:", time.time());

            if (time.time() < 7.4) {
                updateGyro();
                gyroFirst = angles.firstAngle;
                kicker.setPosition(0.45);

                if (time.time() < 5) {
                    if (colour.red() < 10) {
                        blueOnLeft = true;
                        telemetry.addData("Blue location:", "Left");
                    } else if (colour.red() > 10) {
                        blueOnLeft = false;
                        telemetry.addData("Blue location:", "Right");
                    } else {
                        telemetry.addData("Blue location:", "Unknown");
                    }
                }


                if (time.time() > 3.5 && time.time() < 5) {
                    moveDropKick(0.1);
                } else if (time.time() > 5 && time.time() < 7.4) {
                    moveDropKick(0);
                }
            } else {
                kicker.setPosition(0);
            }


            if (blueOnLeft) {
                if (time.time() > 5 && time.time() < 6) {
                    setMoveRobot(-0.25, 0, 0);
                    telemetry.addData("I like", "spagety");

                } else if (time.time() > 6 && time.time() < 7) {
                    setMoveRobot(0.25, 0, 0);

                } else if (time.time() > 7 && time.time() < 7.4) {

                    setMoveRobot(0, 0, 0);
                }

            } else {
                if (time.time() > 5 && time.time() < 6) {
                    setMoveRobot(0.25, 0, 0);
                    telemetry.addData("I like", "nudls");

                } else if (time.time() > 6 && time.time() < 7) {
                    setMoveRobot(-0.25, 0, 0);

                } else if (time.time() > 7 && time.time() < 7.4) {
                    setMoveRobot(0, 0, 0);

                }
            }

            if (getPositionalData()) {
                telemetry.addData("WORK?", vuMark);


                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    vuMarkInt = 2;


                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    vuMarkInt = 1;

                } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                    vuMarkInt = 0;


                }

            } else {
                telemetry.addData("vuMark", "not found.");
            }

            if (vuMarkInt == 0) {
                telemetry.addData("Vumark", "left");
                if (time.time() > 11.5 && time.time() < 12.3) {
                    setMoveRobot(-1, 0, 0);
                }
                if (time.time() > 12.3 && time.time() < 15.5) {
                    setMoveRobot(0, 0, 0);
                }

            } else if (vuMarkInt == 1) {
                telemetry.addData("Vumark", "center");
                if (time.time() > 11.5 && time.time() < 12.4) {
                    setMoveRobot(-1, 0, 0);
                }
                if (time.time() > 12.4 && time.time() < 15.5) {
                    setMoveRobot(0, 0, 0);
                }
            } else if (vuMarkInt == 2) {

                telemetry.addData("Vumark", "right");
                if (time.time() > 11.5 && time.time() < 12.5) {
                    setMoveRobot(-1, 0, 0);
                }
                if (time.time() > 12.5 && time.time() < 15.5) {
                    setMoveRobot(0, 0, 0);
                }
            }

            updateGyro();


            if (time.time() > 16) {
                updateGyro();
                if (angles.firstAngle > gyroFirst - 90) {
                    updateGyro();
                    setMoveRobot(0, 0, 0.3);

                } else if (angles.firstAngle <= gyroFirst - 90) {
                    setMoveRobot(0, 0, 0);
                }

            }
            telemetry.addData("Gryo Heading", angles.firstAngle);


            //Turn the robot so the conveyor faces the cryptobox.
            if (time.time() > 18 && time.time() < 25) {
                moveConveyor(0.05);
            } else if (time.time() > 25) {
                moveConveyor(0);
            }
            telemetry.update();
        }
        telemetry.update();


    }
}