package org.firstinspires.ftc.teamcode.Libraries;

import android.sax.TextElementListener;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
//import org.firstinspires.ftc.teamcode.Autonomous.BlueFind;


/**
 * Created by Ari on 13-11-17.
 * Edited by Harrison on 29-11-17.
 */

public abstract class LibraryBaseAutonomous extends LinearOpMode {
    public static DcMotor topLeft = null;
    public static DcMotor topRight = null;
    public static DcMotor backLeft = null;
    public static DcMotor backRight = null;

    // Define other motors.
    public static DcMotor conveyor = null;
    public static DcMotor intake = null;
    public static DcMotor dropKick = null;

    public static Servo kicker = null;

    public static ColorSensor colour;
    public boolean blueOnLeft;

    public static int vuMarkInt;




    // Gyro stuff.
    private static BNO055IMU imu = null;
    public static Orientation angles;
    public static double gyroyaw = 0;

    // Quick way of checking if the motors *should* be init'ed.
    private boolean baseInited = false;

    // Axial / Lateral / Yaw values.
    private double driveAxial;
    private double driveLateral;
    private double driveYaw;

    // Begin the pain that is Vuforia.

    private static final double ON_AXIS = 10;    // How close we need to be of the target centre-line (Y axis) in mm.
    private static final double CLOSE_ENOUGH = 20;    // How close we need to be of the target standoff (X axis) in mm.

    // Camera direction.
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;

    // The rate we should move on different axis. Needs to be configured.
    public static final double YAW_GAIN = 0.018;  // Rate of correcting heading (angle).
    public static final double LATERAL_GAIN = 0.0027; // Rate of correcting lateral (Y axis).
    public static final double AXIAL_GAIN = 0.0017; // Rate of correct distance (X axis).

    public static VuforiaTrackables relicTrackables;
    public static VuforiaTrackable relicTemplate;
    public static RelicRecoveryVuMark vuMark;
    public static OpenGLMatrix pose;
    public static OpenGLMatrix location;

    public static final int CAMERA_FORWARD_DISPLACEMENT  = 150; // Camera is 110mm in front of robot centre.
    public static final int CAMERA_VERTICAL_DISPLACEMENT = 142; // Camera is 200mm above the ground.
    public static final int CAMERA_LEFT_DISPLACEMENT     = 12;   // Camera is ON the robots centre line.


    // Setup navigation data variables. Only used if targetFound == true. Assigned values later.
    private boolean             targetFound;     // true if Vuforia has found a target.
    private String              targetName;      // the name of the current tracked target.
    private double              robotX;          // X / axial displacement of the robot (forward direction. The more negative the number, the further away the robot is.
    private double              robotY;          // Y / lateral displacement of the robot (left/right direction. If the target is on the right, Y is positive.)
    private double              robotBearing;    // Robot's rotation around the Z axis. (CCW is positive.)
    private double              targetBearing;   // Heading of the target, relative to the phone origin (e.g the phone facing forward, not taking into account robotBearing.)
    private double              relativeBearing; // Heading to the target from the robot's current bearing (targetBearing - robotBearing) (E.g a +ve bearing means the robot must turn CCW to point at the image.)
    private double              targetRange;     // Range from robot's centre to target, in mm.


    // Init the base, and reset all the motors.
    public void initBase() {
        telemetry.addData("STATUS: ", "Initialising Base.");
        topLeft = hardwareMap.dcMotor.get("topLeft");
        topRight = hardwareMap.dcMotor.get("topRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        kicker = hardwareMap.servo.get("kicker");

        colour = hardwareMap.colorSensor.get("colour");

        topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        resetAllMotors(true); // Change to true when we get all 4 motor encoders connected
        baseInited = true;

    }

    public void initConveyor() {
        telemetry.addData("STATUS: ", "Initialising Conveyor.");

        conveyor = hardwareMap.dcMotor.get("conveyor");
        intake = hardwareMap.dcMotor.get("intake");
        dropKick = hardwareMap.dcMotor.get("dropKick");

        conveyor.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        dropKick.setDirection(DcMotorSimple.Direction.FORWARD);

        resetMotor(conveyor, false);
        resetMotor(intake, false);
        resetMotor(dropKick, false);
    }

    public void setMoveRobot(double axial, double lateral, double yaw) {
        setAxial(axial);
        setLateral(lateral);
        setYaw(yaw);
        moveRobot();
    }

    public void moveRobot() {
        // Calculate the required motor speeds to achieve axis motions.
        // Remember, motors spin CCW.

        double topLeftPower = driveYaw - driveAxial - driveLateral; // Top left motor.
        double topRightPower = driveYaw + driveAxial - driveLateral; // Top right motor.
        double backLeftPower = driveYaw - driveAxial + driveLateral; // Bottom left motor.
        double backRightPower = driveYaw + driveAxial + driveLateral; // Bottom right power.


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
    public void moveDropKick(double power){
        dropKick.setPower(power);
        telemetry.addData("DropKick Power" ,power);
    }



    // Create the functions that we use to set variables. Also, sets the range that the variable can be set to.
    public void setAxial(double axial) {
        driveAxial = Range.clip(axial, -1, 1);
    }

    public void setLateral(double lateral) {
        driveLateral = Range.clip(lateral, -1, 1);
    }

    public void setYaw(double yaw) {
        driveYaw = Range.clip(yaw, -1, 1);
    }


    // Quick way of resetting a motor.
    public void resetMotor(DcMotor motor, boolean withEncoder) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Reset all the motors for the holonomic base.
    public void resetAllMotors(boolean withEncoder) {
        if (baseInited) {

            resetMotor(topLeft, true);
            resetMotor(topRight, true);
            resetMotor(backLeft, true);
            resetMotor(backRight, true);

        } else {
            telemetry.addData("ERROR: ", "Motors not defined. Failed resetting.");
            telemetry.update();
        }
    }


    public void initGyro() {
        telemetry.addData("STATUS: ", "Initialising Gyro.");
        BNO055IMU.Parameters gyroParam = new BNO055IMU.Parameters();
        gyroParam.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParam.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParam.calibrationDataFile = "AdafruitIMUCalibration.json";
        gyroParam.loggingEnabled = true;
        gyroParam.loggingTag = "Gyro";
        gyroParam.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParam);
    }

    // updateGyro must be called every update loop.
    public void updateGyro() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void moveConveyor(double power) {
        conveyor.setPower(power);
        telemetry.addData("Conveyor Power: ", power);
    }

    public void moveIntake(double power) {
        intake.setPower(power);
        telemetry.addData("Intake Power:", power);
    }





    public void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Ac2ZR6T/////AAAAGdIyuKX5yU1WhsL+sBQlI9QhbPH4vz8oCEvf34gr7LGyWt0mzfDJahzBJldwHZZZ/SfMij+6i19yz3xkhQ03sTVqAcrlFwAxPLfU6SWVGub0SKiCPzVVB53l+RruAGNUPRL2jDjBg5LccPCWnFBW5R9ISdxzOo1diqV0uMjIlT46GNuPBXIW56uWkOhtZQLk/dm/0f7TRdsoyFoeE/2E4NIzLH7W/tDfm/q3dlwedS1lVdLXPQ/3dQHDOxf++hECRwuSSOPRfoxKlxr1e31nomJFQN2i/KegOBWV4FmaQpKPx9hj33GNeOHW/I4ode7KeEJaUEijd8HQUncF9dwry7YSoCGF7WiYnAPOvM+eZ17s";

        parameters.cameraDirection = CAMERA_CHOICE; // Set the camera that we want to use.
        parameters.useExtendedTracking = false; // Extended tracking is used for augmented reality purposes, we don't want it.

        // Create the Vuforia Localizer.
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        /**
         * Create a transformation describing where the phone is on the robot.
         *
         * The co-ordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing the X axis. The LEFT and RIGHT side of the robot is facing the Y axis.
         * Z is UP on the robot. We mainly focus on the rotation of the Z axis though.
         *
         * The phone starts out lying flat, with the screen facing upwards and with the physical top of the phone pointing to the LEFT side of the robot.
         * If we consider that the camera and screen will be in Landscape Mode, the upper portion of the screen (the right side of the phone) is closest to the front of the robot.
         *
         * If using the rear camera:
         * We need to rotate the camera around it's long axis (the right side of the phone) to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis.
         *
         * If using the front camera:
         * We need to rotate the camera around it's long axis (the left side of the phone) to bring the front camera forward.
         * This requires a positive 90 degree rotation on the Y axis.
         *
         * Next, translate the CAMERA LENS to where it is on the robot. This needs to be done in terms of where the lens is.
         * Ideally, we would want the camera lens to be centre on the robot.
         *`
         * In this code (which requires a lot of changing), the lens is centered, but it is 110 mm forward of the middle of the robot, and 200mm above ground level.
         */

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(
                        // Once again, we want the axes to be in a fixed point. Hence, extrinsic.
                        AxesReference.EXTRINSIC, AxesOrder.YZX,
                        // For this, we check if the selected camera is the front one. If so, we rotate +ve 90 degrees, otherwise -90 degrees.
                        AngleUnit.DEGREES, CAMERA_CHOICE == VuforiaLocalizer.CameraDirection.FRONT ? 90 : -90, 0, 0));

        ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection); // Sets the targets information about the phone location.


    }

    public void activateTracking(){
        relicTrackables.activate();
    }



    public boolean getPositionalData(){
        vuMark = RelicRecoveryVuMark.from(relicTemplate);

        /*if (vuMark != RelicRecoveryVuMark.UNKNOWN){
            telemetry.addData("VuMark ", "is visible", vuMark);
            pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)relicTemplate.getListener(); // See if Vuforia can currently see the target.
            location = listener.getUpdatedRobotLocation();
            VectorF trans = location.getTranslation();
            Orientation rot = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            robotX = (trans.get(0) + 915); // First value of trans is the distance along the X axis the robot is from the target. We add 915 so we move 915mm away from the target.
            robotY = (trans.get(1) + 1195); // Second value of trans is the distance along the Y axis the robot is from the target. We add 1195 so we move 1195mm away from the target.
            // Robot bearing (with +ve being CCW) is defined by the rotation along the Z axis.
            robotBearing = rot.thirdAngle; // The third angle of rot is the rotation along the Z axis.
            // targetRange is based on the distance from the robot to the target, in mm.
            targetRange = Math.hypot(robotX, robotY);
            // targetBearing is based on the angle formed between the X axis and the target range line, FROM the target, hence sin. Doesn't take into account where the phone is currently facing.
            // It's negative because you need to take into account the fact that moving CW is negative, hence you need a negative angle.
            targetBearing = Math.toDegrees(-Math.asin(robotY / targetRange));
            // relativeBearing is the bearing that the robot needs to rotate to face the target, taking into account the angle the camera is already facing.
            relativeBearing = targetBearing - robotBearing;
            */
        telemetry.update();
        if (vuMark != RelicRecoveryVuMark.UNKNOWN){
            return true;
        }
        else {
            return false;
        }

    }

    public boolean cruiseControl(double standOffDistance) {
        boolean closeEnough;

        // Priority #1 - Rotate to always be pointing at the target. (YAW - Z rotation)
        double Y = ( relativeBearing * YAW_GAIN ); // We want to get relativeBearing to 0.

        // Priority #2 - Drive laterally to be centred with the target (LATERAL - Y axis)
        double L = ( robotY * LATERAL_GAIN ); // We want to get robotY to 0.

        // Priority #3 - Drive forward until we reach the desired standOffDistance from the target. (AXIAL - X axis)
        // Remember that robotX is a negative value, hence we switch it to positive here.
        // If you are confused, you can instead do (-robotX - standOffDistance) - it's the same thing.
        double A = ( -(robotX + standOffDistance) * AXIAL_GAIN );

        // Send these values to Vuforia_OmniDrive.
        setMoveRobot(A, L, Y);

        closeEnough = ( ( Math.abs( robotX + standOffDistance ) < CLOSE_ENOUGH ) && // If our current robotX (distance) is within 20 mm of standOffDistance
                ( Math.abs( robotY ) < ON_AXIS )); // (logical) and we are within 10mm of the centre of the target, then true.

        return (closeEnough);

    }

    public void addNavTelemetry() {
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("Visible:", targetName);
            telemetry.addData("RobotX:", robotX);
            telemetry.addData("RobotY:", robotY);
            telemetry.addData("robotBearing:", robotBearing);
            telemetry.addData("targetBearing:", targetBearing);
            telemetry.addData("relativeBearing:", relativeBearing);
        }
        else {

            telemetry.addData("No ", "vuMark found.");
        }
    }
}