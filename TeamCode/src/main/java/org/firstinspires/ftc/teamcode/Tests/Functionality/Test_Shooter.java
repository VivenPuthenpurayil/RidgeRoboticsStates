package org.firstinspires.ftc.teamcode.Tests.Functionality;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.Test;


@Autonomous(name = "shooter", group = "Smart")

public class Test_Shooter extends Test {

    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException{
        setRob(new Rover(hardwareMap, runtime, this, Rover.setupType.none));
        setRuntime(runtime);


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            if (gamepad1.a) {
                rob.rack.setPower(0.4);
            }
            else if (gamepad1.y){
                rob.rack.setPower(-0.4);
            }
            else {
                rob.rack.setPower(0);
            }


        }
    }

    public DcMotor motor(String name, DcMotor.Direction direction) throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);

        return motor;
    }
}
