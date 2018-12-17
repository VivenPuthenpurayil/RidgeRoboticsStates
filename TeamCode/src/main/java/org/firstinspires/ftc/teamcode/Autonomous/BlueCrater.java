package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

@Autonomous(name="Blue Crater", group ="Smart")
public class BlueCrater extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Rover.setupType.drive, Rover.setupType.latching, Rover.setupType.vuforia, Rover.setupType.phoneswivel);

        while (opModeIsActive()) {
            telemetry.addLine("Deploy time");
            telemetry.update();
            rob.deploy();
            sampling();

            break;









           /* angleOfLander();
            rob.driveTrainEncoderMovement(0.5, 15, 10, 1, Rover.movements.forward);
            sampling();
            goToCraterOtherSide();*/
        }
    }
}


