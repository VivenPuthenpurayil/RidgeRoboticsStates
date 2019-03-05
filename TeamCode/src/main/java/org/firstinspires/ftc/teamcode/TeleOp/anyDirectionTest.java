package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

import static org.firstinspires.ftc.teamcode.Control.Constants.ROTATION_SPEED;


@TeleOp(name = "DriveModeAngle", group = "Smart")

public class anyDirectionTest extends TeleOpControl {


    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode() throws InterruptedException{

        setup(runtime, Rover.setupType.drive);


        while (opModeIsActive()) {

            // GAMEPAD OBJECT
            standardGamepadData();

            if (validStick(xAxis1, yAxis1)) { //MAIN DIRECTIONS

                double angle = (Math.PI *2.0 + Math.atan2(yAxis1, -xAxis1))%(Math.PI * 2.0);
                telemetry.addData("Angle: ", Math.toDegrees(angle));
                telemetry.update();
                rob.driveTrainMovementAngleRadians(Math.hypot(yAxis1, xAxis1), angle);


            }
            else {
                rob.stopDrivetrain();
            }


        }
    }

}
