package org.firstinspires.ftc.teamcode.Tests.Functionality;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.Test;


@Autonomous(name = "Super IMU Swing Test", group = "Test")

public class Test_SimpleSuperIMUSwingTest extends Test {

    public ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException{
        setup(runtime, Rover.setupType.drive, Rover.setupType.imu);

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            rob.driveTrainIMUSwingTurnMovementOrig(0.4, Rover.movements.forward, 3000, 90, 0.02, Rover.turnside.cw);
        }

    }





}
