package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;

@Autonomous(name="Blue Depot", group ="Smart")
public class BlueDepot extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    public enum pos{
        left, right, center, unknown
    }
    pos o = pos.unknown;
    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Rover.setupType.drive, Rover.setupType.latching, Rover.setupType.phoneswivel, Rover.setupType.tensorflow, Rover.setupType.imu);

        rob.servo.setPosition(0.43);

        while (opModeIsActive()) {
            telemetry.addLine("Deploy time");
            telemetry.update();
            rob.deploy();
            //sampling();
            while (rob.vuforia.tfod != null && opModeIsActive() && o == pos.unknown) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = rob.vuforia.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }


                        int x =  rob.checkMinerals(goldMineralX, silverMineral1X, silverMineral2X);
                        telemetry.addData("Value: ", x);
                        switch (x){
                            case 0:
                                telemetry.addData("Gold Mineral Position", "Right");
                                o = pos.right;
                                break;
                            case 1:
                                if (goldMineralX < silverMineral2X){
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    o = pos.left;
                                }
                                else{
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    o = pos.center;
                                }
                                break;
                            case 2:
                                if (goldMineralX < silverMineral1X){
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    o = pos.left;
                                }
                                else{
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    o = pos.center;
                                }
                                break;
                            default:
                                telemetry.addLine("no clue b");
                                o = pos.unknown;
                                break;
                        }

                        /*
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                          if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                          } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                          } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                          }
                        }*/
                    }
                    telemetry.update();
                }
            }

            rob.driveTrainEncoderMovement(0.5, 5, 1, 300, Rover.movements.left);
            rob.turn(90, Rover.turnside.ccw, 0.3, Rover.axis.center);

            switch (o){
                case left:
                    rob.driveTrainEncoderMovement(0.5, 3, 1.5, 50, Rover.movements.left);
                    break;

                case right:
                    rob.driveTrainEncoderMovement(0.5, 3, 1.5, 50, Rover.movements.right);

                default:
                    rob.driveTrainEncoderMovement(0.5, 3, 2, 50, Rover.movements.forward);
                    break;

            }

            break;









           /* angleOfLander();
            rob.driveTrainEncoderMovement(0.5, 15, 10, 1, Rover.movements.forward);
            sampling();
            goToCraterOtherSide();*/
        }
    }
}


