package org.firstinspires.ftc.teamcode.Tests.System;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.Test;


@Autonomous(name = "Encoder Momements Test", group = "Test")

public class Test_Encoders extends Test {

    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException{
        setup(runtime, Rover.setupType.drive);



        while (opModeIsActive()) {


            telemetry.addLine("Beginning");
            telemetry.update();
            for (int i = 0; i < allMovements.length; i++) {
                telemetry.addLine(allMovements[i].name());
                telemetry.update();
                rob.driveTrainEncoderMovement(0.5, 10, 10, 500, allMovements[i]);
            }

            telemetry.addLine("End");
            telemetry.update();
            sleep(2000);


        }
    }


}
