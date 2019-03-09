package org.firstinspires.ftc.teamcode.Tests.System;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.Test;


@Autonomous(name = "IMU Turn Test", group = "Test")

public class Test_IMUTurn extends Test {

    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException{
        setup(runtime, Rover.setupType.imu, Rover.setupType.drive);


        while (opModeIsActive()) {
            telemetry.addData("Orientation: ", rob.imu.getAngularOrientation());
            telemetry.update();

            rob.turn(90, Rover.turnside.ccw, 0.4, Rover.axis.center);
            sleep(1000);
            rob.turn(90, Rover.turnside.cw, 0.4, Rover.axis.center);
            sleep(1000);


        }
    }


}
