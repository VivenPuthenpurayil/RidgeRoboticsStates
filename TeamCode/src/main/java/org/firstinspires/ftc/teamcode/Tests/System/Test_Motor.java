package org.firstinspires.ftc.teamcode.Tests.System;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

@TeleOp(name = "Motor Functions Test", group = "Test")
public class Test_Motor extends TeleOpControl {
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motor = hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setRob(new Rover(hardwareMap, runtime, this));
        setRuntime(runtime);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            standardGamepadData();

            if (gamepad1.a){ // motor test
                telemetry.addLine("Motor Test");
                telemetry.update();
                for (int i = 0; i < 5 && opModeIsActive(); i++) {
                    motor.setPower(0.2 * (i+1));
                    sleep(300);
                }
                motor.setPower(0);
            }
            else if (gamepad1.y){ //encoder test
                telemetry.addLine("Encoder Test");
                telemetry.update();
                rob.encoderMovement(0.8, 1, 10, 0, Rover.movements.rackExtend, motor);
            }
        }

    }
    public void encoder(DcMotor motor) throws InterruptedException {




    }
}
