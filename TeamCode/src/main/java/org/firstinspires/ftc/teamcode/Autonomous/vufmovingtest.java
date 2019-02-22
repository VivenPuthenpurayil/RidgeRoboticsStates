package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

@Autonomous(name="testvufmove", group ="Smart")
public class vufmovingtest extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Rover.setupType.vuforia, Rover.setupType.drive, Rover.setupType.phoneswivel);
        while (opModeIsActive()) {
            double[] x = {20,16,10};
            rob.moveusingvuf(new Rover.Position(x,90));
            sleep(10000);
        }
    }
}