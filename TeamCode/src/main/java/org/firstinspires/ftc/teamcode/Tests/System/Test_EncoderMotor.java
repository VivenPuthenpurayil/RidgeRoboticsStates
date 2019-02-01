package org.firstinspires.ftc.teamcode.Tests.System;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

@TeleOp(name = "Encoders Functions Test", group = "Test")
public class Test_EncoderMotor extends TeleOpControl {
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        setRob(new Rover(hardwareMap, runtime, this, Rover.setupType.drive));
        setRuntime(runtime);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            standardGamepadData();


            if (gamepad1.y){ //encoder test
                telemetry.addLine("Encoder Test");
                telemetry.update();
                rob.driveTrainEncoderMovement(0.8, 1, 10, 0, Rover.movements.forward);
            }
            else if (gamepad1.a){
                telemetry.addLine("Encoder Test");
                telemetry.update();
                rob.driveTrainEncoderMovement(0.2, 1, 10, 0, Rover.movements.forward);
            }
        }

    }
    public void encoder(DcMotor motor) throws InterruptedException {

    }
}
