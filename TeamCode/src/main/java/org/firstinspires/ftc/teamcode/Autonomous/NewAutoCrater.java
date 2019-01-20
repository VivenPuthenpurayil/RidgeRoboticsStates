package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

public class NewAutoCrater extends AutonomousControl {

    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Rover.setupType.autonomous);

        rob.deploy();

        rob.sample();
        sampling();

    }
}