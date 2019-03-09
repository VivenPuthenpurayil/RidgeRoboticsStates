package org.firstinspires.ftc.teamcode.Autonomous.States;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.VuforiaHandler;

import java.util.List;

import static org.firstinspires.ftc.teamcode.Control.VuforiaHandler.LABEL_GOLD_MINERAL;

@Autonomous(name="Blue Crater", group ="Smart")
public class BlueCrater extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    public enum pos {
        left, right, center, unknown
    }

    private pos o = pos.unknown;


    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Rover.setupType.drive, Rover.setupType.latching, Rover.setupType.phoneswivel, Rover.setupType.imu, Rover.setupType.tensorflow);



        telemetry.addLine("Ready!");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            telemetry.addLine("Deploy time");
            telemetry.update();


            rob.deploy();

            rob.turn(40, Rover.turnside.cw, 0.6, Rover.axis.center);

            if (rob.vuforia.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                boolean obj = false;
                while (!obj && opModeIsActive()) {
                    rob.driveTrainMovement(0.05, Rover.movements.ccw);
                    List<Recognition> updatedRecognitions = rob.vuforia.tfod.getRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() > 0) {
                            telemetry.addData("recognized", updatedRecognitions.get(0).getLabel());
                        }
                        telemetry.addData("oboy: ", o);
                        telemetry.update();
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                o = pos.right;
                                obj = true;
                                rob.stopDrivetrain();
                            } else {
                                obj = true;
                                rob.stopDrivetrain();
                                rob.turn(45, Rover.turnside.cw, 0.6, Rover.axis.center);
                                rob.driveTrainEncoderMovement(0.4, 0.4, 3, 30, Rover.movements.left);
                            }
                        }

                    }
                    telemetry.addData("obj: ", o);
                    telemetry.update();
                }
                if (o == pos.unknown) {
                    obj = false;

                    while (!obj && opModeIsActive()) {
                        rob.driveTrainMovement(0.05, Rover.movements.cw);
                        List<Recognition> updatedRecognitions = rob.vuforia.tfod.getRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() > 0) {
                                telemetry.addData("recognized", updatedRecognitions.get(0).getLabel());
                            }
                            telemetry.addData("oboy: ", o);
                            telemetry.update();
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    obj = true;
                                    rob.stopDrivetrain();
                                    o = pos.center;
                                } else {
                                    obj = true;
                                    rob.stopDrivetrain();
                                    o = pos.left;
                                }
                            }

                        }
                        telemetry.addData("obj: ", o);
                        telemetry.update();
                    }
                }
                telemetry.addData("o: ", o);
                telemetry.update();
                switch (o) {
                    case center:
                        rob.driveTrainEncoderMovement(0.8, 1.5, 5, 500, Rover.movements.forward);
                        break;
                    case left:
                        rob.turn(45, Rover.turnside.cw, 0.3, Rover.axis.center);
                        rob.driveTrainEncoderMovement(0.8, 2, 5, 500, Rover.movements.forward);
                        break;
                    case right:
                        rob.driveTrainEncoderMovement(0.8, 2, 5, 500, Rover.movements.forward);
                        break;
                    default:
                        rob.driveTrainEncoderMovement(0.8, 2, 5, 500, Rover.movements.forward);
                        break;
                }

            }


        }
        if (rob.vuforia.tfod != null) {
            rob.vuforia.tfod.shutdown();
        }
    }


    public int checkMinerals(int gold, int sil1, int sil2) {
        if (gold == -1 && sil1 != -1 && sil2 != -1) {
            return 0;
        } else if (gold != -1 && sil1 == -1 && sil2 != -1) {
            return 1;
        } else if (gold != -1 && sil1 != -1 && sil2 == -1) {
            return 2;
        } else {
            return -1;
        }

    }
}
