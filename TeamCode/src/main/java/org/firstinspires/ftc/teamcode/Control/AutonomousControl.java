package org.firstinspires.ftc.teamcode.Control;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetectorNarrow;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetectorNarrow.GoldLocation.CENTER;
import static com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetectorNarrow.GoldLocation.LEFT;
import static com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetectorNarrow.GoldLocation.RIGHT;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.Control.Constants.mmPerInch;

public abstract class AutonomousControl extends Central {



    public void angleOfLander() throws InterruptedException {
        double x = runtime.seconds();
        while (runtime.seconds() <= x + 3 &&opModeIsActive()) {
            while (!rob.vuforia.checkVisibility().equals("false") && opModeIsActive()) {
                VectorF translation = rob.vuforia.lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(rob.vuforia.lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle,
                        rotation.thirdAngle);
                telemetry.update();
                if (rotation.thirdAngle > 25 && opModeIsActive()) {
                    rob.driveTrainMovement(0.2, Rover.movements.cw);
                    telemetry.addLine("CW");
                    telemetry.update();


                }else if(rotation.thirdAngle < 20 && opModeIsActive()){
                    rob.driveTrainMovement(0.2, Rover.movements.ccw);
                    telemetry.addLine("CCW");
                    telemetry.update();

                }
                else {
                    rob.stopDrivetrain();
                    break;
                }



            }


        }

    }

    public void centerPosition() throws InterruptedException {
        rob.vuforia.checkVisibility();
        VectorF translation = rob.vuforia.lastLocation.getTranslation();
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

        // express the rotation of the robot in degrees.
        Orientation rotation = Orientation.getOrientation(rob.vuforia.lastLocation, EXTRINSIC, XYZ, DEGREES);
        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

        telemetry.update();
        if (!((translation.get(0) > 50)&&(translation.get(0) < 55))) {
            rob.driveTrainMovement(0.2, Rover.movements.right);
        } else {
            rob.stopDrivetrain();
        }

    }

    public void leftPosition() throws InterruptedException {
        rob.vuforia.checkVisibility();
        VectorF translation = rob.vuforia.lastLocation.getTranslation();
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

        // express the rotation of the robot in degrees.
        Orientation rotation = Orientation.getOrientation(rob.vuforia.lastLocation, EXTRINSIC, XYZ, DEGREES);
        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

        telemetry.update();
        if (!((translation.get(0) > 30)&&(translation.get(0) < 35))) {
            rob.driveTrainMovement(0.2, Rover.movements.right);
        } else {
            rob.stopDrivetrain();
        }

    }

    public void rightPosition() throws InterruptedException {
        rob.vuforia.checkVisibility();
        VectorF translation = rob.vuforia.lastLocation.getTranslation();
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

        // express the rotation of the robot in degrees.
        Orientation rotation = Orientation.getOrientation(rob.vuforia.lastLocation, EXTRINSIC, XYZ, DEGREES);
        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

        telemetry.update();
        if (!((translation.get(0) > 70)&&(translation.get(0) < 75))) {
            rob.driveTrainMovement(0.2, Rover.movements.right);
        } else {
            rob.stopDrivetrain();
        }

    }

    public void knockingOffCenter() throws InterruptedException{
        rob.driveTrainEncoderMovement(0.2, 10,10, 2, Rover.movements.forward);
        //drop off the team marker stuff isn't configured/made
    }

    public void knockingOffLeft() throws InterruptedException{
        rob.driveTrainEncoderMovement(0.2, 5,10, 2, Rover.movements.forward);
        rob.driveTrainEncoderMovement(0.2, 5,10, 2, Rover.movements.right);
        //drop off the team marker stuff isn't configured/made
    }

    public void knockingOffRight() throws InterruptedException{
        rob.driveTrainEncoderMovement(0.2, 5,10, 2, Rover.movements.forward);
        rob.driveTrainEncoderMovement(0.2, 5,10, 2, Rover.movements.left);
        //drop off the team marker stuff isn't configured/made
    }

    public void goToCraterCraterSide() throws InterruptedException{
        rob.turn(45, Rover.turnside.ccw,.5, Rover.axis.back);
        rob.driveTrainEncoderMovement(0.5, 144, 20, 1, Rover.movements.backward);
    }

    public void goToCraterOtherSide() throws InterruptedException{
        rob.turn(45, Rover.turnside.cw,.5, Rover.axis.back);
        rob.driveTrainEncoderMovement(0.5, 144, 20, 1, Rover.movements.backward);
    }

    public void sampling () throws InterruptedException {
        SamplingOrderDetector detector = new SamplingOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        detector.downscale = 0.4;
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();

        telemetry.addData("Sample: ", detector.getCurrentOrder().name());
        telemetry.update();
        /*switch(detector.getCurrentOrder()){
            case LEFT:
                leftPosition();
                knockingOffLeft();
            case CENTER:
                centerPosition();
                knockingOffCenter();
            case RIGHT:
                rightPosition();
                knockingOffRight();
            case UNKNOWN:
                sampling();
        }*/

    }
    public void samplingNarrow () throws InterruptedException {
        SamplingOrderDetectorNarrow detector = new SamplingOrderDetectorNarrow();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        detector.downscale = 0.4;
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();
        int leftScore = 0;
        int centerScore = 0;
        int rightScore = 0;
        double startTime= runtime.milliseconds();

        while (runtime.milliseconds() < startTime + 5000){
            switch (detector.getCurrentOrder()){
                case LEFT:
                    leftScore++;
                    break;
                case RIGHT:
                    rightScore++;
                    break;
                case CENTER:
                    centerScore++;
                    break;
                case NOTLEFT:
                    leftScore--;
                    break;
                case NOTRIGHT:
                    rightScore--;
                    break;
                case NOTCENTER:
                    centerScore--;
                    break;
                default: break;
            }
        }
        int max = Math.max(Math.max(centerScore,leftScore),rightScore);
        if(max == centerScore){
            telemetry.addData("detected:", "center");
            telemetry.update();
            centerPosition();
            knockingOffCenter();
        }
        if(max == leftScore){
            telemetry.addData("detected:", "left");
            telemetry.update();
            leftPosition();
            knockingOffLeft();
        }
        if(max==rightScore){
            telemetry.addData("detected:", "right");
            telemetry.update();
            rightPosition();
            knockingOffRight();
        }



    }




}
