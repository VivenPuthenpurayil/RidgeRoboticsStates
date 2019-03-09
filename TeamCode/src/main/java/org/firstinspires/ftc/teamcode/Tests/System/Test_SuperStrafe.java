package org.firstinspires.ftc.teamcode.Tests.System;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;


@Autonomous(name = "Super Strafe Test", group = "Test")

public class Test_SuperStrafe extends TeleOpControl {

    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException{
        setup(runtime, Rover.setupType.drive);



        while (opModeIsActive()) {


            telemetry.addLine("Beginning");
            telemetry.update();
            rob.driveTrainEncoderSuperStrafeMovement(0.3, 1, 10, 300, Rover.movements.forward, 1, Rover.movements.cw);

            telemetry.addLine("End");
            telemetry.update();
            sleep(2000);


        }
    }


}
