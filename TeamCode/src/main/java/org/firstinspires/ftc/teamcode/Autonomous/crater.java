package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

@Autonomous(name="Crater", group ="Smart")



public class crater extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Rover.setupType.autonomous);

        while (opModeIsActive()) {
            rob.deploy();
            //do vuforia
            sampling();
            rob.driveTrainEncoderMovement(0.2, 10, 10, 2, Rover.movements.backward);
            rob.turn(45,Rover.turnside.ccw,0.2,Rover.axis.center );
            while(rob.rangeDistancefront() > 3 && opModeIsActive()){
                rob.driveTrainEncoderMovement(0.3, 8.5, 6, 200, Rover.movements.forward);
            }
            rob.turn(90,Rover.turnside.ccw,0.2,Rover.axis.center );
            while(rob.rangeDistancefront() > 24 && opModeIsActive()){
                rob.driveTrainEncoderMovement(0.3, 8.5, 6, 200, Rover.movements.forward);
            }
            //deposit
            while(rob.rangeDistanceback() > 46 && opModeIsActive()){
                rob.driveTrainEncoderMovement(0.3, 8.5, 6, 200, Rover.movements.backward);
            }
        }
    }
}
