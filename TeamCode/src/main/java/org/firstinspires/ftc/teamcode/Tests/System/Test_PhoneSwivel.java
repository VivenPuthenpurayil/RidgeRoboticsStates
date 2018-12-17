package org.firstinspires.ftc.teamcode.Tests.System;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.Test;


@Autonomous(name = "Phone Swivel Test", group = "Test")

public class Test_PhoneSwivel extends Test {

    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException{
        setup(runtime, Rover.setupType.phoneswivel);


        while (opModeIsActive()) {




        }
    }


}
