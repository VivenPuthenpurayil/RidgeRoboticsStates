package org.firstinspires.ftc.teamcode.Tests.Functionality;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.TeleOpControl;


@Autonomous(name = "Two Motor Test", group = "Smart")

public class Test_TwoMotor extends TeleOpControl {

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{

        setRuntime(runtime);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            standardGamepadData();

        }
    }

}
