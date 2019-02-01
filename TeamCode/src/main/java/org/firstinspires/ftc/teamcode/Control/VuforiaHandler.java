package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
import static org.firstinspires.ftc.teamcode.Control.Constants.CAMERA_CHOICE;
import static org.firstinspires.ftc.teamcode.Control.Constants.VUFORIA_KEY;
import static org.firstinspires.ftc.teamcode.Control.Constants.mmFTCFieldWidth;
import static org.firstinspires.ftc.teamcode.Control.Constants.mmTargetHeight;


public class VuforiaHandler {

    public int CAMERA_FORWARD_DISPLACEMENT = 0, CAMERA_VERTICAL_DISPLACEMENT = 0, CAMERA_LEFT_DISPLACEMENT = 0;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    public TFObjectDetector tfod;

    public OpenGLMatrix blueRoverLocationOnField, redFootprintLocationOnField, frontCratersLocationOnField, backSpaceLocationOnField, phoneLocationOnRobot;
    public VuforiaTrackable blueRover, redFootprint, frontCraters, backSpace;
    public VuforiaTrackables targetsRoverRuckus;
    public List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    public OpenGLMatrix lastLocation = null;
    public boolean targetVisible = false;
    public VuforiaLocalizer vuforia;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public enum type{
        images, minerals, both;
    }
    public VuforiaHandler(Central central, type setups) {
        hardwareMap = central.hardwareMap;
        telemetry = central.telemetry;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        if (setups == type.images || setups == type.both) {
            // Load the data sets that for the trackable objects. These particular data
            // sets are stored in the 'assets' part of our application.
            targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
            blueRover = targetsRoverRuckus.get(0);
            blueRover.setName("Blue-Rover");
            redFootprint = targetsRoverRuckus.get(1);
            redFootprint.setName("Red-Footprint");
            frontCraters = targetsRoverRuckus.get(2);
            frontCraters.setName("Front-Craters");
            backSpace = targetsRoverRuckus.get(3);
            backSpace.setName("Back-Space");

            // For convenience, gather together all the trackable objects in one easily-iterable collection */

            allTrackables.addAll(targetsRoverRuckus);


            /**
             * In order for localization to work, we need to tell the system where each target is on the field, and
             * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
             * Transformation matrices are a central, important concept in the math here involved in localization.
             * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
             * for detailed information. Commonly, you'll encounter transformation matrices as instances
             * of the {@link OpenGLMatrix} class.
             *
             * If you are standing in the Red Alliance Station looking towards the center of the field,
             *     - The X axis runs from your left to the right. (positive from the center to the right)
             *     - The Y axis runs from the Red Alliance Station towards the other side of the field
             *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
             *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
             *
             * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
             *
             * Before being transformed, each target image is conceptually located at the origin of the field's
             *  coordinate system (the center of the field), facing up.
             */

            /**
             * To place the BlueRover target in the middle of the blue perimeter wall:
             * - First we rotate it 90 around the field's X axis to flip it upright.
             * - Then, we translate it along the Y axis to the blue perimeter wall.
             */
            blueRoverLocationOnField = OpenGLMatrix
                    .translation(0, mmFTCFieldWidth, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
            blueRover.setLocation(blueRoverLocationOnField);

            redFootprintLocationOnField = OpenGLMatrix
                    .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
            redFootprint.setLocation(redFootprintLocationOnField);

            frontCratersLocationOnField = OpenGLMatrix
                    .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
            frontCraters.setLocation(frontCratersLocationOnField);

            backSpaceLocationOnField = OpenGLMatrix
                    .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
            backSpace.setLocation(backSpaceLocationOnField);

            /**
             * Create a transformation matrix describing where the phone is on the robot.
             *
             * The coordinate frame for the robot looks the same as the field.
             * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
             * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
             *
             * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
             * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
             * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
             *
             * If using the rear (High Res) camera:
             * We need to rotate the camera around it's long axis to bring the rear camera forward.
             * This requires a negative 90 degree rotation on the Y axis
             *
             * If using the Front (Low Res) camera
             * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
             * This requires a Positive 90 degree rotation on the Y axis
             *
             * Next, translate the camera lens to where it is on the robot.
             * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
             */

            CAMERA_FORWARD_DISPLACEMENT = 0;   // eg: Camera is 110 mm in front of robot center
            CAMERA_VERTICAL_DISPLACEMENT = 0;   // eg: Camera is 200 mm above ground
            CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

            phoneLocationOnRobot = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                            CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

            /**  Let all the trackable listeners know where the phone is.  */
            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            }

        }

        if (setups == type.minerals || setups == type.both){
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }
        }


    }
    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public VuforiaHandler(Central central, boolean tensor){
        hardwareMap = central.hardwareMap;
        telemetry = central.telemetry;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }
    public String checkVisibility() throws InterruptedException{
        targetVisible = false;
        String r = "false";
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                r = trackable.getName();
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }
        return r;
    }
}

