package org.firstinspires.ftc.teamcode.Tests.System;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.Test;


@Autonomous(name = "IMU Test", group = "Test")

public class Test_IMU extends Test {

    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException{
        setup(runtime, Rover.setupType.imu);


        while (opModeIsActive()) {


            telemetry.addLine("Beginning");
            telemetry.update();
            telemetry.addData("Status: ", rob.imu.getSystemStatus());
            telemetry.update();
            telemetry.addData("Orientation: ", rob.imu.getAngularOrientation());
            telemetry.update();

        }
    }


}
