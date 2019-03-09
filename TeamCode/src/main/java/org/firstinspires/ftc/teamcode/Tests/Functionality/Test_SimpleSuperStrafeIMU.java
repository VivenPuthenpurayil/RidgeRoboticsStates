package org.firstinspires.ftc.teamcode.Tests.Functionality;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.Test;


@Autonomous(name = "Super IMU Strafe Test", group = "Test")

public class Test_SimpleSuperStrafeIMU extends Test {

    public ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException{
        setup(runtime, Rover.setupType.drive, Rover.setupType.imu);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
       //    rob.driveTrainIMUSuperStrafeMovement(0.4, Rover.movements.forward, 3000, 90, 0.01, Rover.turnside.cw);
            rob.driveTrainMovementAngle(0.2, 90);
        }

    }





}
