package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.LibraryBaseTeleOP;

/**
 * Created by Ari on 13-11-17.
 */

@TeleOp(name= "Encoder Test", group = "Testing")
public class Encoder_Driving extends LibraryBaseTeleOP {

    private static double driveAxial = 0;
    private static double driveLateral = 0;
    private static double driveYaw = 0;

    private static double updAxial = 0;
    private static double updLateral = 0;


    private static double k = 0.1;
    private static double rotation;
    private static boolean hasSet;
    private static double originalRotation;
    private static double updatedRotation;
    private static double axial;
    private static double lateral;
    private static double yaw;




    @Override
    public void init(){
        initBase();
        initGyro();
        telemetry.update();
    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        updateGyro();
        if (gamepad1.right_stick_x == 0) {
            if (gamepad1.right_bumper) { // For testing purposes

                if (!hasSet) {
                    originalRotation = angles.firstAngle;
                    hasSet = true;


                    if (gamepad1.right_bumper) {
                        updateGyro();
                        rotation = angles.firstAngle;
                        updatedRotation = rotation - originalRotation;

                        double sin = Math.sin(-updatedRotation * 0.0174533);
                        double cos = Math.cos(-updatedRotation * 0.0174533);

                        driveAxial = -gamepad1.left_stick_y;
                        driveLateral = gamepad1.left_stick_x;

                        axial = ((driveLateral * sin) + (driveAxial * cos));
                        lateral = ((driveLateral * cos) + (driveAxial * sin));

                        double topLeftPower = -axial - lateral; // Top left motor.
                        double topRightPower = axial - lateral; // Top right motor.
                        double backLeftPower = axial + lateral; // Bottom left motor.
                        double backRightPower = axial + lateral; // Bottom right power.
                        // Normalise all the motor speeds to no values exceed 100% (which is really 1.0)
                        // The /= compound operator means divide the left value by the right, and assign that number into the left value.
                        double max = Math.max(Math.abs(topLeftPower), Math.abs(topRightPower)); // See which value is larger: topLeft or topRight.
                        max = Math.max(max, Math.abs(backLeftPower)); // See which value is larger: The above max or bottomLeftPower
                        max = Math.max(max, Math.abs(backRightPower)); // See which value is larger: The above max or bottomRightPower
                        if (max > 1.0) { // If the max value is greater than 1, then for each variable, divide it by the "max" and set it to that value.
                            topLeftPower /= max;
                            topRightPower /= max;
                            backLeftPower /= max;
                            backRightPower /= max;
                        }

                        topLeft.setPower(topLeftPower);
                        topRight.setPower(topRightPower);
                        backLeft.setPower(backLeftPower);
                        backRight.setPower(backRightPower);

                        telemetry.addData("topLeft: ", topLeft.getPower());
                        telemetry.addData("topRight: ", topRight.getPower());
                        telemetry.addData("backLeft: ", backLeft.getPower());
                        telemetry.addData("backRight: ", backRight.getPower());
                    }
                    telemetry.addData("Original Rotation: ", originalRotation);
                    telemetry.addData("Current Rotation: ", rotation);
                    telemetry.addData("K Value:", k);
                }
            } else {
                hasSet = false;
            }
            if (gamepad1.dpad_up) {
                k = k + 0.1;
            } else if (gamepad1.dpad_down) {
                k = k - 0.1;
            } else if (gamepad1.dpad_right) {
                k = k + 0.01;
            } else if (gamepad1.dpad_left) {
                k = k - 0.01;
            }

            telemetry.update();

        }
    }
 

    @Override
    public void stop(){
        telemetry.addData("Final K Value: ", k);
        telemetry.update();
    }
}
