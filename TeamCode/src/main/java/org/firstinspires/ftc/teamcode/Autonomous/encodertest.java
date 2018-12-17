package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

@Autonomous(name="Encodertest", group ="Smart")


public class encodertest extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Rover.setupType.drive);
        rob.driveTrainEncoderMovement(0.5,1/24.0,5, 30, Rover.movements.forward);
        sleep(5000);
        rob.driveTrainEncoderMovement(0.5,1/24.0,5, 30, Rover.movements.right);
        sleep(5000);
        rob.driveTrainEncoderMovement(0.5,360/245.0,5, 30, Rover.movements.cw);
        sleep(5000);


    }
}