package org.firstinspires.ftc.teamcode.Tests.Functionality;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;
@Autonomous(name="Double Camera Test", group ="Camera")
public class Test_DoubleCamera extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Rover.setupType.fullvision);



        telemetry.addLine("Ready!");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            telemetry.addLine("Deploy time");
            telemetry.update();


        }
        if (rob.vuforia.tfod != null) {
            rob.vuforia.tfod.shutdown();
        }
    }

}
