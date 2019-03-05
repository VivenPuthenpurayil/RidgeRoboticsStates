package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

@Autonomous(name="Depot", group ="Smart")

public class Depot extends AutonomousControl {

    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Rover.setupType.autonomous);

        while (opModeIsActive()){
            rob.deploy();
            //do vuforia
            sampling();
            rob.driveTrainEncoderMovement(0.2, 10,10, 2, Rover.movements.backward);
            rob.turn(90,Rover.turnside.ccw,0.2,Rover.axis.center );

            while(rob.rangeDistanceFRU() > 3 && opModeIsActive()){
                rob.driveTrainEncoderMovement(0.3, 3, 6, 200, Rover.movements.forward);

            }

            rob.turn(135,Rover.turnside.cw,0.2,Rover.axis.center );


            while(rob.rangeDistanceFLU() > 24 && opModeIsActive()){
                rob.driveTrainEncoderMovement(0.3, 3, 6, 200, Rover.movements.forward);

            }

            sleep(1000);

            while(rob.rangeDistanceBCU() > 46 && opModeIsActive()){

                rob.driveTrainEncoderMovement(0.3, 3, 6, 200, Rover.movements.backward);
            }



        }
    }
}
