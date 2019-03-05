package org.firstinspires.ftc.teamcode.Tests.System;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;


@Autonomous(name = "Arm Speeds Test", group = "Test")

public class Test_ArmSpeeds extends TeleOpControl {

    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException{
        setup(runtime, Rover.setupType.mineralControl);


        while (opModeIsActive()) {

            standardGamepadData();
            if (validStick(xAxis1, yAxis1)) { //MAIN DIRECTIONS
                rob.arm.setPower(yAxis1/Math.abs(yAxis1) * 0.1);
            }else {
                rob.arm.setPower(0);
            }
            if (gamepad1.left_trigger > 0.25){
                rob.linear.setPower(0.9);
            }
            else if (gamepad1.right_trigger > 0.25){
                rob.linear.setPower(-0.9);
            }
            else {
                rob.linear.setPower(0);
            }

            if (gamepad1.dpad_up){
                rob.collector.setPower(0.7);
            }
            else if (gamepad1.dpad_down){
                rob.collector.setPower(-0.7);
            }
            else {
                rob.collector.setPower(0);
            }
        }
    }


}
