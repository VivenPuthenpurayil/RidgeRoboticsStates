package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

@Autonomous(name="servotest", group ="Smart")
public class servotest extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime,  Rover.setupType.phoneswivel);

        while (opModeIsActive()) {
            rob.servo.setPosition(0);
             sleep(1000);
                rob.servo.setPosition(0.3);
                sleep(1000);
                rob.servo.setPosition(0.5);
                sleep(1000);
                rob.servo.setPosition(0.8);
                sleep(1000);
                rob.servo.setPosition(1);
                sleep(1000);

        }
    }
}