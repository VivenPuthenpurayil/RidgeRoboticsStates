
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

@Autonomous(name="servotester", group ="Smart")


public class servotest extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();
double x = 0.47;
    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Rover.setupType.phoneswivel);
        while (opModeIsActive() && x< 1) {
            telemetry.addData("position %f",  rob.servo.getPosition());
             x+= 0.01;
            rob.servo.setPosition(x);
            telemetry.update();
            sleep(100);



        }
    }
}