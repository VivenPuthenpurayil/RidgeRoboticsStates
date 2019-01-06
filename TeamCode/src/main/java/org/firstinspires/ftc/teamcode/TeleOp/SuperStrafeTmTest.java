package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

@TeleOp(name = "StrafeTesT", group = "Smart")
public class SuperStrafeTmTest extends TeleOpControl {

        private ElapsedTime runtime = new ElapsedTime();
        public void runOpMode() throws InterruptedException{

            setup(runtime/*,Rover.setupType.drive, Rover.setupType.latching*/, Rover.setupType.imu);


            while (opModeIsActive()) {

                // GAMEPAD OBJECT
                standardGamepadData();
                telemetry.addData("1:",rob.superstrafe(Math.toDegrees(Math.atan(yAxis1/xAxis1)),Math.sqrt(yAxis1*yAxis1+(xAxis1*xAxis1)),xAxis2)[0]);
                telemetry.addData("2:",rob.superstrafe(Math.toDegrees(Math.atan(yAxis1/xAxis1)),Math.sqrt(yAxis1*yAxis1+(xAxis1*xAxis1)),xAxis2)[1]);

                /*if (validStick(xAxis1, yAxis1)) { //MAIN DIRECTIONS
                    rob.superstrafe(Math.toDegrees(Math.atan(yAxis1/xAxis1)),Math.sqrt(yAxis1*yAxis1+(xAxis1*xAxis1)),xAxis2);
                } else {
                    rob.stopDrivetrain();
                }


                if(gamepad1.a){
                    rob.rack.setPower(1);
                }
                else if (gamepad1.y){
                    rob.rack.setPower(-1);
                }
                else {
                    rob.rack.setPower(0);
                }*/
            }
        }
}

