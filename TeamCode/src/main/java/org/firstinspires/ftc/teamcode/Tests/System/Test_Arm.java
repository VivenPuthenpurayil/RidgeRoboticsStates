package org.firstinspires.ftc.teamcode.Tests.System;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;


@Autonomous(name = "Arm Test", group = "Test")

public class Test_Arm extends TeleOpControl {

    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException{
        setup(runtime, Rover.setupType.mineralControl);


        while (opModeIsActive()) {

            standardGamepadData();
            if (gamepad1.dpad_down){
                rob.arm.setPower(0.3);
            }
            else if (gamepad1.dpad_up){
                rob.arm.setPower(-0.1);
            }
            else {
                rob.arm.setPower(0);
            }



            if (gamepad1.b){
                rob.linear.setPower(0.4);
            }
            else if (gamepad1.x){
                rob.linear.setPower(-0.4);
            }
            else {
                rob.linear.setPower(0);
            }

            if (gamepad1.a){
                rob.collector.setPower(0.7);
            }
            else if (gamepad1.y){
                rob.collector.setPower(-0.7);
            }
            else {
                rob.collector.setPower(0);
            }
        }
    }


}
