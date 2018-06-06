package org.firstinspires.ftc.teamcode.VuforiaCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

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
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Ari on 01-06-17.
 * Credit to gearsincorg for sample code. You can find his original sample here: https://github.com/gearsincorg/FTCVuforiaDemo
 *
 * This file is not an OpMode.
 * Instead, it sets up and configures Vuforia. It also gathers the variables that we require.
 * It does not drive the robot!
 *
 * Once a target is found, it displays all relevant information as telemetry.
 *
 * When approaching a target, there are three priorities:
 * #1 - Rotate the robot towards the target for target retention.
 * #2 - Drive laterally (sideways) based on the distance off centre (Y axis, with the target on the RIGHT of the robot being +ve)
 * #3 - Drive axially (forward) until we reach the requested standoff distance (depends on our button-pressing mechanism)
 *
 */
public class Vuforia_Navigation {
    /*
     * Begin by creating / setting all of our variables.
     * Setup our constant integers.
     */
    private static final int    MAX_TARGETS  = 4;     // The number of Vuforia targets there are.
    private static final double ON_AXIS      = 10;    // How close we need to be of the target centre-line (Y axis) in mm.
    private static final double CLOSE_ENOUGH = 20;    // How close we need to be of the target standoff (X axis) in mm.

    // Select the camera we want to use. For us, it is the BACK camera. Alternate is FRONT.
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;

    // The rate we should move on different axis. Needs to be configured.
    public  static final double YAW_GAIN     = 0.018;  // Rate of correcting heading (angle).
    public  static final double LATERAL_GAIN = 0.0027; // Rate of correcting lateral (Y axis).
    public  static final double AXIAL_GAIN   = 0.0017; // Rate of correct distance (X axis).

    // Private class members. The variables are not assigned values yet, this is done later.
    private LinearOpMode        myOpMode; // Create a variable that stores an opMode.
    //private Vuforia_OmniDrive   myRobot;  // Allows us to access the motors in Robot_OmniDrive
    private VuforiaTrackables   targets;  // List of target images.

    // Setup navigation data variables. Only used if targetFound == true. Assigned values later.
    private boolean             targetFound;     // true if Vuforia has found a target.
    private String              targetName;      // the name of the current tracked target.
    private double              robotX;          // X / axial displacement of the robot (forward direction. The more negative the number, the further away the robot is.
    private double              robotY;          // Y / lateral displacement of the robot (left/right direction. If the target is on the right, Y is positive.)
    private double              robotBearing;    // Robot's rotation around the Z axis. (CCW is positive.)
    private double              targetBearing;   // Heading of the target, relative to the phone origin (e.g the phone facing forward, not taking into account robotBearing.)
    private double              relativeBearing; // Heading to the target from the robot's current bearing (targetBearing - robotBearing) (E.g a +ve bearing means the robot must turn CCW to point at the image.)
    private double              targetRange;     // Range from robot's centre to target, in mm.

    // Other stuff
    private boolean             showDebugTelemetry = true; // SET HERE. Shows the value of robotX, robotY and robotZ.
    public static final int CAMERA_FORWARD_DISPLACEMENT  = 150; // Camera is 110mm in front of robot centre.
    public static final int CAMERA_VERTICAL_DISPLACEMENT = 142; // Camera is 200mm above the ground.
    public static final int CAMERA_LEFT_DISPLACEMENT     = 12;   // Camera is ON the robots centre line.

    /*
     * Begin code by creating a constructor. Used to create an object from this class in Vuforia_TeleOp.
     * Set all the variables to null / zero.
     */

    public Vuforia_Navigation(){

        targetFound = false;
        targetName = null;
        targets = null;

        robotX = 0;
        robotY = 0;
        robotBearing = 0;
        targetBearing = 0;
        relativeBearing = 0;
        targetRange = 0;
    }


    /*
     * Create a function that sends telemetry about the target tracking. Note I have no idea how this formatting works.
     */
    public void addNavTelemetry(){
        showDebugTelemetry=true;
        if (targetFound && showDebugTelemetry ){
            myOpMode.telemetry.addData("DEBUG: ", "ON");
            myOpMode.telemetry.addData("Visible:", targetName);
            myOpMode.telemetry.addData("RobotX:", robotX);
            myOpMode.telemetry.addData("RobotY:", robotY);
            myOpMode.telemetry.addData("robotBearing:", robotBearing);
            myOpMode.telemetry.addData("targetBearing:", targetBearing);
            myOpMode.telemetry.addData("relativeBearing:", relativeBearing);
        }
        else if (targetFound && !showDebugTelemetry){ // If Vuforia has found a target and we don't want to debug.
            myOpMode.telemetry.addData("Visible: ", targetName); // The name of the currently active target.
            myOpMode.telemetry.addData("Robot: ", "[X]:[Y] (B) [%5.0fmm]:[%5.0fmm) (%4.0f°)",
                    robotX, robotY, robotBearing); // The X, Y and Z (rotation) values, I assume rounded to 5, 5, and 4 decimal places, respectively.

            myOpMode.telemetry.addData("Target: ", "[R] (B):(RB) [%5.0fmm] (%4.0f°):(%4.0f°)",
                    targetRange, targetBearing, relativeBearing); // The X, Y and Z (rotation) values of the target distance, target bearing and relativeBearing.

            // How much Yaw (Z rotation) movement is required to get relativeBearing to 0.
            // Note, ? and : functions work like this.... (expression) ? (true) : (false)
            myOpMode.telemetry.addData("Turn: ", "%s %4.0f°",  relativeBearing < 0 ? ">>> CW " : "<<< CCW", Math.abs(relativeBearing));

            // How much Lateral (Y axis) movement is required to get robotY to 0.
            myOpMode.telemetry.addData("Strafe: ", "%s %5.0fmm", robotY < 0 ? "LEFT" : "RIGHT", Math.abs(robotY));

            // How much Axial (X axis) movement is required to get robotX to 0. Note that robotX will always be negative.
            myOpMode.telemetry.addData("- Distance", "%5.0fmm", Math.abs(robotX));
            }

        else { // if Vuforia has not found a target...
            myOpMode.telemetry.addData("Visible: ", "No Targets Found");
        }

    }

    /*
     * Start tracking the targets.
     */
    public void activateTracking(){
        // Start tracking any defined targets.
        if (targets != null){
            targets.activate();
        }
    }

    /***
     * Set the Axial, Lateral and Yaw axis values that are required to get to the target.
     *
     * @param standOffDistance for how close we get the centre of the robot to the target (in mm)
     *                         Remember, standOffDistance is from the centre, not from where the phone is.
     * @return true if we are 'closeEnough' to the target.
     *
     * We need to get X, Y and Z basically equal to zero. However, we don't want to turn too fast at any one time, hence it is multiplied (basically divided) by ***_GAIN
     */
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
        myRobot.setYaw(Y);
        myRobot.setAxial(A);
        myRobot.setLateral(L);

        closeEnough = ( ( Math.abs( robotX + standOffDistance ) < CLOSE_ENOUGH ) && // If our current robotX (distance) is within 20 mm of standOffDistance
                ( Math.abs( robotY ) < ON_AXIS )); // (logical) and we are within 10mm of the centre of the target, then true.

            return (closeEnough);

    }

    /***
     * Initialise Vuforia and the navigation interface.
     *
     * @param opMode - points toward the opMode that is calling this function. (Vuforia_TeleOp)
     * @param robot - points toward the robot hardware (Vuforia_OnniDrive)
     *
     */
    public void initVuforia(LinearOpMode opMode, Vuforia_OmniDrive robot){

        // Save reference to the opMode and Hardware. Used in other parts of this code.
        myOpMode = opMode;
        myRobot = robot;

        /**
         * Init Vuforia, either giving it an id of the camera monitor (so you can see the camera on the phone), or not.
         * Also, set the camera that we want to use.
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters( R.id.cameraMonitorViewId ); // Allows us to see the camera view on the phone.
        // For better performance, use this line:
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Vuforia license key. https://developer.vuforia.com/license-manager if you don't have one.
        parameters.vuforiaLicenseKey = "Ac2ZR6T/////AAAAGdIyuKX5yU1WhsL+sBQlI9QhbPH4vz8oCEvf34gr7LGyWt0mzfDJahzBJldwHZZZ/SfMij+6i19yz3xkhQ03sTVqAcrlFwAxPLfU6SWVGub0SKiCPzVVB53l+RruAGNUPRL2jDjBg5LccPCWnFBW5R9ISdxzOo1diqV0uMjIlT46GNuPBXIW56uWkOhtZQLk/dm/0f7TRdsoyFoeE/2E4NIzLH7W/tDfm/q3dlwedS1lVdLXPQ/3dQHDOxf++hECRwuSSOPRfoxKlxr1e31nomJFQN2i/KegOBWV4FmaQpKPx9hj33GNeOHW/I4ode7KeEJaUEijd8HQUncF9dwry7YSoCGF7WiYnAPOvM+eZ17s";

        parameters.cameraDirection = CAMERA_CHOICE; // Set the camera that we want to use.
        parameters.useExtendedTracking = false; // Extended tracking is used for augmented reality purposes, we don't want it.

        // Create the Vuforia Localizer.
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data files that we wish to track.
         * These data files are found in the "assets" folder.
         * They represent the four image targets we wish to track.
         */
        targets = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        targets.get(0).setName("Blue Near: Wheels" );
        targets.get(1).setName("Red Far: Tools"    );
        targets.get(2).setName("Blue Far: Legos"   );
        targets.get(3).setName("Red Near: Gears"   );

        // For convenience, gather together all the trackable objects in one easily-iterable array.
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets); // Add all the targets into the array.

        /**
         * THE NEXT TWO MATRIX'S REQUIRE EDITING.
         * Currently, this creates a translation / rotation matrix to be used for all images.
         * It puts the images 150mm above the 0:0:0 origin, but it rotates it so it faces the -X axis.
         * THIS IS WRONG! All images are in the same exact place. They need to be correctly moved into their specific places, but this works for now.
         */

        OpenGLMatrix targetOrientation = OpenGLMatrix // Create a matrix
                .translation(0, 0, 150) // Translate the images 150mm upwards (Z axis)
                .multiplied(Orientation.getRotationMatrix(
                        /**
                         * Extrinsic rotation is rotation around a fixed axis. Intrinsic is around something that is moving, e.g the robot.
                         * As we are talking about the target, we want it to be Extrinsic.
                         * Images start our flat, laying face-down. In order to flip it to face outwards, we rotate it 90 degrees on the X axis.
                         * We then rotate it -90 degrees on the Z axis to make it face the -X axis.
                         * Confused? Watch this video: https://www.youtube.com/watch?v=T3-F2xpaesg
                         * AxesOrder just sets the order that the axes should be set in.
                         */

                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, -90)); // Rotate it 90 degrees on the X axis, and then -90 degrees on the Z axis.


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

        // Set all the targets to be in the same location, and have the same camera orientation.
        // We will need to change the location of the targets. Not sure about the camera orientation though.

        for (VuforiaTrackable trackable : allTrackables){
            trackable.setLocation(targetOrientation); // Set the location to targetOrientatin
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection); // Sets the targets information about the phone location.
        }
    }

    /**
     * Check if any of the targets are visible.
     *
     * @return true if any are found.
     */
    public boolean targetsAreVisible(){

        int targetTestID = 0;

        // Check each target in turn, but stop looking when the first target is found.
        while (( targetTestID < MAX_TARGETS ) && !targetIsVisible(targetTestID)){
            targetTestID++;
        }

        return (targetFound);
    }

    /**
     * Determine if the specified targetTestID is visible
     * If it is, retrieve the relevant data, then calculate the robot and target locations.
     *
     * @param targetId - variable from targetsAreVisible()
     * @return - true if the specified target is found.
     */
    public boolean targetIsVisible(int targetId){

        VuforiaTrackable target = targets.get(targetId); // Get the name / info of the target.
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener(); // See if Vuforia can currently see the target.
        OpenGLMatrix location = null;

        // If we have a target, look for an updated robot position.
        // if ((target exists) and (the target is registered) and (the target is visible){
        if ((target != null) && (listener != null) && listener.isVisible()){
            targetFound = true;
            targetName = target.getName();

            // If we have an updated robot location, update all the relevant tracking info.
            location = listener.getUpdatedRobotLocation();
            if (location != null) { // if Vuforia gets an updated robot position.

                // Create a translation and rotation vector for the robot.
                VectorF trans = location.getTranslation(); // Gets the translation (x and y) of the robot.
                Orientation rot = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES); // Gets the rotations of all the axes. In theory, only the Z axis should be anything apart from zero.

                // Robot position is defined by the Matrix translation.
                robotX = trans.get(0); // First value of trans is the distance along the X axis the robot is from the target.
                robotY = trans.get(1); // Second value of trans is the distance along the Y axis the robot is from the target.
                // We add 200(mm) to this number because it is REALLY off (but always by 200mm)

                // Robot bearing (with +ve being CCW) is defined by the rotation along the Z axis.
                robotBearing = rot.thirdAngle; // The third angle of rot is the rotation along the Z axis.

                // targetRange is based on the distance from the robot to the target, in mm.
                targetRange = Math.hypot(robotX, robotY);

                // targetBearing is based on the angle formed between the X axis and the target range line, FROM the target, hence sin. Doesn't take into account where the phone is currently facing.
                // It's negative because you need to take into account the fact that moving CW is negative, hence you need a negative angle.
                targetBearing = Math.toDegrees(-Math.asin(robotY / targetRange));

                // relativeBearing is the bearing that the robot needs to rotate to face the target, taking into account the angle the camera is already facing.
                relativeBearing = targetBearing - robotBearing;
            }
            targetFound = true;

        }
        else {
            // Indicate that there is no target visible.
            targetFound = false;
            targetName = "None";
        }

        return targetFound;

    }

}
