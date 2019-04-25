package org.firstinspires.ftc.teamcode.Autonomous.States;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

@Autonomous(name="Blue Crater", group ="Smart")
public class BlueCrater extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    public enum pos {
        left, right, center, unknown
    }

    private pos o = pos.unknown;


    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Rover.setupType.drive, Rover.setupType.latching, Rover.setupType.phoneswivel, Rover.setupType.imu, Rover.setupType.mineralControl, Rover.setupType.tensorflow, Rover.setupType.sensors);



        telemetry.addLine("Ready!");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            telemetry.addLine("Deploy time");
            telemetry.update();


            int position = rob.deploy();
            telemetry.addData("position: ", position);
            telemetry.update();

            switch (position){
                case 0:
                default:
                    rob.driveTrainEncoderMovement(0.7, 2.5, 5, 30, Rover.movements.bl);
                    rob.driveTrainEncoderMovement(0.7, 2.5, 5, 30, Rover.movements.br);
                    rob.turn(136, Rover.turnside.ccw, 0.8, Rover.axis.center);
                    while (rob.BRU.getDistance(DistanceUnit.INCH) > 5){
                        rob.driveTrainMovement(0.8, Rover.movements.right);
                    }
                    rob.driveTrainEncoderMovement(0.7, 2.5, 5, 0, Rover.movements.forward);
                    rob.stopDrivetrain();
                    //rob.anyMovementTime(0.2, Rover.movements.armDown, 100, rob.arm);
                    rob.anyMovementTime(0.8, Rover.movements.linearOut, 1000, rob.linear);
                    rob.anyMovementTime(0.8, Rover.movements.collectorCollect, 1000, rob.collector);
                    rob.anyMovementTime(0.8, Rover.movements.linearIn, 1000, rob.linear);
                    rob.driveTrainEncoderMovement(0.8, 4.5, 5, 30, Rover.movements.backward);
                    break;
                case 1:
                    rob.driveTrainEncoderMovement(0.7, 0.5, 5, 0, Rover.movements.forward);
                    rob.driveTrainEncoderMovement(0.7, 2, 5, 0, Rover.movements.left);
                    rob.driveTrainEncoderMovement(0.7, 1.5, 5, 0, Rover.movements.right);
                    rob.driveTrainEncoderMovement(0.7, 2.5, 5, 0, Rover.movements.backward);
                    rob.turn(135, Rover.turnside.ccw, 0.8, Rover.axis.center);
                    while (rob.BRU.getDistance(DistanceUnit.INCH) > 5){
                        rob.driveTrainMovement(0.4, Rover.movements.right);
                    }
                    rob.driveTrainEncoderMovement(0.7, 2.5, 5, 0, Rover.movements.forward);
                    rob.stopDrivetrain();
                    //rob.anyMovementTime(0.2, Rover.movements.armDown, 100, rob.arm);
                    rob.anyMovementTime(0.8, Rover.movements.linearOut, 1000, rob.linear);

                    rob.anyMovementTime(0.4, Rover.movements.collectorCollect, 1000, rob.collector);
                    rob.anyMovementTime(0.8, Rover.movements.linearIn, 1000, rob.linear);
                    rob.driveTrainEncoderMovement(0.8, 4, 5, 30, Rover.movements.backward);
                    break;
                case 2:
                    rob.driveTrainEncoderMovement(0.7, 0.9, 5, 30, Rover.movements.forward);
                    rob.driveTrainEncoderMovement(0.7, 2.5, 5, 30, Rover.movements.tl);
                    rob.driveTrainEncoderMovement(0.7, 2.5, 5, 30, Rover.movements.br);
                    rob.driveTrainEncoderMovement(0.7, 3.5, 5, 30, Rover.movements.backward);
                    rob.turn(135, Rover.turnside.ccw, 0.8, Rover.axis.center);
                    while (rob.BRU.getDistance(DistanceUnit.INCH) > 4 && rob.FRU.getDistance(DistanceUnit.INCH) > 14){
                        rob.driveTrainMovement(0.8, Rover.movements.right);
                    }
                    rob.driveTrainEncoderMovement(0.7, 2, 5, 0, Rover.movements.forward);

                    // rob.anyMovementTime(0.2, Rover.movements.armDown, 100, rob.arm);
                    rob.anyMovementTime(0.8, Rover.movements.linearOut, 1000, rob.linear);
                    rob.anyMovementTime(0.4, Rover.movements.collectorCollect, 2000, rob.collector);
                    rob.anyMovementTime(0.8, Rover.movements.linearIn, 1000, rob.linear);
                    rob.driveTrainEncoderMovement(0.8, 5.25, 5, 30, Rover.movements.backward);

                    break;

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
