

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@TeleOp(name = "Vuforia Servo", group = "Concept")
public class VuforiaServoMove extends AutonomousControl {

    static final double MAX_POS = 0.6;     // Maximum rotational position
    static final double MIN_POS = 0.2;     // Minimum rotational position

    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;

    // Define class members
    Servo servo;
    double position = 0;

    boolean targetVisible;


    @Override
    public void runOpMode() throws InterruptedException {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servo = hardwareMap.get(Servo.class, "kobe");
        setRob(new Rover(hardwareMap, runtime, this, Rover.setupType.vuforia));
        rob.vuforia.targetsRoverRuckus.activate();
        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            String image = "NONE";
            targetVisible = false;
            for (VuforiaTrackable trackable : rob.vuforia.allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    image = trackable.getName();

                    if (robotLocationTransform != null) {
                        rob.vuforia.lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = rob.vuforia.lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(rob.vuforia.lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                // Display the current value
                telemetry.addData("Servo Position", "%5.2f", position);
                telemetry.addData(">", "Press Stop to end test.");

                if (image.equals(rob.vuforia.frontCraters.getName())) {
                    double angle = Math.atan(translation.get(1) / (72-Math.abs(translation.get(0))));
                    telemetry.addData("Angle: ", Math.toDegrees(angle));
                    telemetry.update();
                    position = angle / Math.PI;
                    servo.setPosition(0.4 + position);
                    sleep(100);
                    idle();

                }else if(image.equals(rob.vuforia.blueRover.getName())) {
                    double angle = Math.atan(translation.get(0) / (72-Math.abs(translation.get(1))));
                    telemetry.addData("Angle: ", Math.toDegrees(angle));
                    telemetry.update();
                    position = angle / Math.PI;
                    servo.setPosition(0.4 + position);
                    sleep(100);
                    idle();

                }else if (image.equals(rob.vuforia.backSpace.getName())) {
                    double angle = -Math.atan(translation.get(1) / (72-Math.abs(translation.get(0))));
                    telemetry.addData("Angle: ", Math.toDegrees(angle));
                    telemetry.update();
                    position = angle / Math.PI;
                    servo.setPosition(0.4 + position);
                    sleep(100);
                    idle();

                }else if(image.equals(rob.vuforia.redFootprint.getName())) {
                    double angle = -Math.atan(translation.get(0) / (72-Math.abs(translation.get(1))));
                    telemetry.addData("Angle: ", Math.toDegrees(angle));
                    telemetry.update();
                    position = angle / Math.PI;
                    servo.setPosition(0.4 + position);
                    sleep(100);
                    idle();

                }
            }
        }
    }
}