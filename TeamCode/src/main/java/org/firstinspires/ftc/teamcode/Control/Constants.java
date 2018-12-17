package org.firstinspires.ftc.teamcode.Control;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Constants {


    //--------------------------------ENCODERS-------------------------


    public static final double COUNTS_PER_MOTOR_NEVEREST = 1680;
    public static final double COUNTS_PER_MOTOR_TETRIX = 1440;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_NEVEREST * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);                   // Conversion: Encoder Count to Inches
    public static final double COUNTS_PER_TETRIX_INCH = (COUNTS_PER_MOTOR_TETRIX * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);               // Conversion: Encoder Counts Motor Tetrix to Inches


    //--------------------------------TELE-OP VALUES--------------------
    public static final double ROTATION_SPEED = 0.8;
    public static final double DEAD_ZONE_SIZE = 0.1;
    public static final double D_PAD_SPEED = 0.4;
    public static final double CRAWL_SPEED = 0.2;
    public static final double WRIST_SPEED = .01;
    public static final double ELBOW_SPEED = .01;

    //--------------------------------VUFORIA----------------------------

    public static final String VUFORIA_KEY = "AUaoObT/////AAABmZESOvJAgkXJk00eebGyewdT7a9NZK5YLL9rnvWS5jwOFcmnubSqY4E8gnBkljxMEsVfBteT2JE95kMUrCT379Ya4Inep4AQqT2IQRvFD5lTr2PYVIWp9c6oe2f1C9T8M1aco5W/O4kU1kOf0UGikrcSheCnor0nc2siDkbfT8s1YRRZVXl56xrCw7Po6PsU4tkZJ9F2tp9YyMmowziEIjbeLJ+V3C51kRnNyiMuF3ev0Einp5ioXgW82RJMPiJLiiKZZP9ARLYGLsNX+nOFFJXRrKywydpwcWwlyAaHzWpSJ9yWgAMwFY1iN4BBq8VaFInXY+T40/g/WCBxM7WLkWNNOe44921UcyFGgjqxf/T2";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch = 25.4f;
    public static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    public static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;


    //--------------------------------CONFIGURATION VALUES--------------------

    public static final String motorFRS = "motorFR";
    public static final String motorFLS = "motorFL";
    public static final String motorBRS = "motorBR";
    public static final String motorBLS = "motorBL";


    public static final String rackS = "rack";
    public static final String armS = "arm";

    public static final String deployingLimitS = "depLimit";
    public static final String latchingLimitS = "latLimit";

    public static final String imuS = "imu";


    public static final String phoneSwivelS = "phoneSwivel";
    public static final String linearS = "linear";

    public static final String collectorS = "collector";
}
