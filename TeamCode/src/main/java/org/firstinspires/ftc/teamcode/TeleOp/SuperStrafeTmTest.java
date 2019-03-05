package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

@Autonomous(name = "StrafeTesT", group = "Test")
public class SuperStrafeTmTest extends TeleOpControl {

        private ElapsedTime runtime = new ElapsedTime();
        public void runOpMode() throws InterruptedException{

            setup(runtime,Rover.setupType.drive, Rover.setupType.imu);
            while (opModeIsActive()) {
                rob.superstrafe(50,0.5,0.5,Rover.movements.cw);
            }
            rob.stopDrivetrain();
        }
}

