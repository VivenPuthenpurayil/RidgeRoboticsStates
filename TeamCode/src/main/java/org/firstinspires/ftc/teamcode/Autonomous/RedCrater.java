package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

import java.util.List;


@Autonomous(name="Red Crater", group ="Smart")



public class RedCrater extends AutonomousControl {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = " AcowzAr/////AAABmZ1O2qADtkUEtt6Ubzpq/jpUtz5mLmoeN8h5vCHHyrt+z8HddaupaRFcm9GWPN3OyHX0Pu4J4q/AGcE3K3MrXHCzMqReZXCWIRXfbWw5sFOBKA2b9jQ4PFPamJ1OWeZKmKe8xVHmpaC+KN47xMnIsPOg5EUdvoQPbPA9Tot3I4eeGRiJD2NosRcVqYypb/ubtx1v3kfFA5QxC5ob/mbbRMU5z/A0Sk5mzvaDFpR7qnu1/WbwpdM8Uo2HxjzBeTB3+oTIQ3aFMKhPfgQl1KXZkRRbtdfiVopSwkTT5yZh0H7IgZVAwc7Bk726o+h2z8evOSTuDOkps4+8lYLbBrNUVemer37ET+gQLmbOzd4/8d7q";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Rover.setupType.autonomous);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
           // initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {

                //rob.deploy();
/*
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } int x =  checkMinerals(goldMineralX, silverMineral1X, silverMineral2X);
                            telemetry.addData("Value: ", x);
                            switch (x){
                                case 0:
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    break;
                                case 1:
                                    if (goldMineralX < silverMineral2X){
                                        telemetry.addData("Gold Mineral Position", "Left");
                                        break;
                                    }
                                    else{
                                        telemetry.addData("Gold Mineral Position", "Center");
                                        break;
                                    }
                                case 2:
                                    if (goldMineralX < silverMineral1X){
                                        telemetry.addData("Gold Mineral Position", "Left");
                                        break;
                                    }
                                    else{
                                        telemetry.addData("Gold Mineral Position", "Center");
                                        break;
                                    }
                                default:
                                    telemetry.addLine("no clue b");
                                    break;
                            }

                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public int checkMinerals(int gold, int sil1, int sil2){
        if (gold == -1 && sil1 != -1 && sil2 != -1){
            return 0;
        }
        else if (gold != -1 && sil1 == -1 && sil2 != -1){
            return 1;
        }
        else if (gold != -1 && sil1 != -1 && sil2 == -1){
            return 2;
        }else{
            return -1;
        }

    }
    /**
     * Initialize the Vuforia localization engine.
     */
/*
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }
*/
    /**
     * Initialize the Tensor Flow Object Detection engine.

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

    }
}
*/
            }
        }
    }
}
