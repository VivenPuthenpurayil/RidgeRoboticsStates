package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

@Autonomous(name="testauto", group ="Smart")
public class testable extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Rover.setupType.drive, Rover.setupType.latching, Rover.setupType.vuforia);

        while (opModeIsActive()) {

            angleOfLander();

        }
    }
}


