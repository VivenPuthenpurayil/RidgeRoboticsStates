

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.Control.Constants.ROTATION_SPEED;

@TeleOp(name = "Vuforia Servo No Action", group = "Concept")
public class VuforiaServoMove2 extends TeleOpControl {

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
        servo = hardwareMap.get(Servo.class, "servo");
        setRob(new Rover(hardwareMap, runtime, this, Rover.setupType.vuforia, Rover.setupType.drive));
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
                    sleep(100);

                }else if(image.equals(rob.vuforia.blueRover.getName())) {
                    double angle = Math.atan(translation.get(0) / (72-Math.abs(translation.get(1))));
                    telemetry.addData("Angle: ", Math.toDegrees(angle));
                    telemetry.update();
                    position = angle / Math.PI;
                    sleep(100);

                }else if (image.equals(rob.vuforia.backSpace.getName())) {
                    double angle = -Math.atan(translation.get(1) / (72-Math.abs(translation.get(0))));
                    telemetry.addData("Angle: ", Math.toDegrees(angle));
                    telemetry.update();
                    position = angle / Math.PI;
                    sleep(100);

                }else if(image.equals(rob.vuforia.redFootprint.getName())) {
                    double angle = -Math.atan(translation.get(0) / (72-Math.abs(translation.get(1))));
                    telemetry.addData("Angle: ", Math.toDegrees(angle));
                    telemetry.update();
                    position = angle / Math.PI;
                    sleep(100);

                }
            }

            // GAMEPAD OBJECT
            standardGamepadData();


            if (rightStickButtonPressed) {
                // CLOCKWISE
                rob.driveTrainMovement(ROTATION_SPEED, Rover.movements.cw);
            } else if (leftStickButtonPressed) {
                // COUNTERCLOCKWISE
                rob.driveTrainMovement(ROTATION_SPEED, Rover.movements.ccw);
            }
            else if (gamepad1.dpad_up){
                rob.driveTrainMovement(0.2,Rover.movements.forward);
            }
            else if (gamepad1.dpad_right){
                rob.driveTrainMovement(0.2,Rover.movements.right);
            }
            else if (gamepad1.dpad_left){
                rob.driveTrainMovement(0.2,Rover.movements.left);
            }
            else if (gamepad1.dpad_down){

                rob.driveTrainMovement(0.2,Rover.movements.backward);
            }
            else if (validStick(xAxis1, yAxis1)) { //MAIN DIRECTIONS

                if (yAxis1 >= Math.abs(xAxis1)) {
                    rob.driveTrainMovement(fb,Rover.movements.forward);
                    //FORWARD
                }else if (yAxis1 <= -Math.abs(xAxis1)) {
                    rob.driveTrainMovement(fb,Rover.movements.backward);
                    //BACKWARD
                } else if (Math.abs(yAxis1) < xAxis1) {
                    rob.driveTrainMovement(rl,Rover.movements.right);
                    //RIGHT
                } else if (-Math.abs(yAxis1) > xAxis1) {
                    rob.driveTrainMovement(rl,Rover.movements.left);
                    //LEFT
                }
            } else if (validStick(xAxis2, yAxis2)) {    //DIAGONAL

                if (yAxis2 >= 0 && xAxis2 >= 0) {
                    rob.driveTrainMovement(diagonalSpeed,Rover.movements.tr);
                    //TOP RIGHT
                } else if (yAxis2 >= 0 && xAxis2 < 0) {
                    rob.driveTrainMovement(diagonalSpeed,Rover.movements.tl);
                    //TOP LEFT
                } else if (yAxis2 < 0 && xAxis2 >= 0) {
                    rob.driveTrainMovement(diagonalSpeed, Rover.movements.br);
                    //BOTTOM RIGHT
                } else if (yAxis2 < 0 && xAxis2 < 0) {
                    rob.driveTrainMovement(diagonalSpeed,Rover.movements.bl);
                    //BOTTOM LEFT
                }
            } else {
                rob.stopDrivetrain();
            }

            

        }
    }
}