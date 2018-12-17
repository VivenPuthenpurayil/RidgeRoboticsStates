package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

@Autonomous(name="Autonomous Small Bot", group ="Smart")
public class AutonomousSmallBot extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Rover.setupType.drive, Rover.setupType.latching, Rover.setupType.vuforia);

        while (opModeIsActive()) {
            rob.deploy();
            angleOfLander();
            rob.driveTrainEncoderMovement(0.5, 15, 10, 1, Rover.movements.forward);
            sampling();
            goToCraterOtherSide();
        }
    }
}


