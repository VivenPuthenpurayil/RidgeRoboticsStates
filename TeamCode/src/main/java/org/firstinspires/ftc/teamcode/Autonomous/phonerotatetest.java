package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

@Autonomous(name="totatemount", group ="Smart")
public class phonerotatetest extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Rover.setupType.vuforia, Rover.setupType.phoneswivel);
        while (opModeIsActive()) {
rob.phoneSwivel();
            telemetry.addData("orient %f",  rob.offsetservo());
            sleep(500);
        }
    }
}