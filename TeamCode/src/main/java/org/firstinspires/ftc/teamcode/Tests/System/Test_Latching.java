package org.firstinspires.ftc.teamcode.Tests.System;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.Test;


@Autonomous(name = "Latching Test", group = "Test")

public class Test_Latching extends Test {

    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException{
        setup(runtime, Rover.setupType.latching);


        while (opModeIsActive()) {


            telemetry.addLine("Beginning");
            telemetry.update();
            telemetry.addLine("Deploying");
            telemetry.update();
            rob.deploy();
            sleep(1000);
            telemetry.addLine("Latching");
            telemetry.update();
            rob.latch();

            telemetry.addLine("End");
            telemetry.update();
            sleep(2000);


        }
    }


}
