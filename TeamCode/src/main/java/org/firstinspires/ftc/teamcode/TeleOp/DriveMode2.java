package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

import static org.firstinspires.ftc.teamcode.Control.Constants.ROTATION_SPEED;


@TeleOp(name = "DriveMode 2", group = "Smart")

public class DriveMode2 extends TeleOpControl {


    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode() throws InterruptedException{

        setup(runtime, Rover.setupType.drive, Rover.setupType.latchingTele, Rover.setupType.mineralControl);


        while (opModeIsActive()) {

            // GAMEPAD OBJECT
            standardGamepadData();

             if (gamepad1.dpad_up){
                rob.driveTrainMovement(0.2,Rover.movements.forward);
            }
            else if (gamepad1.dpad_right){
                rob.driveTrainMovement(0.2,Rover.movements.right);
            }
            else if (gamepad1.dpad_left){
                rob.driveTrainMovement(0.2,Rover.movements.left);
            }
            else if (gamepad1.dpad_down){

                rob.driveTrainMovement(0.2,Rover.movements.backward);
            }
            else if (validStick(xAxis1, yAxis1)) { //MAIN DIRECTIONS

                if (yAxis1 >= Math.abs(xAxis1)) {
                    rob.driveTrainMovement(fb,Rover.movements.forward);
                    //FORWARD
                }else if (yAxis1 <= -Math.abs(xAxis1)) {
                    rob.driveTrainMovement(fb,Rover.movements.backward);
                    //BACKWARD
                } else if (Math.abs(yAxis1) < xAxis1) {
                    rob.driveTrainMovement(rl,Rover.movements.right);
                    //RIGHT
                } else if (-Math.abs(yAxis1) > xAxis1) {
                    rob.driveTrainMovement(rl,Rover.movements.left);
                    //LEFT
                }
            } else if (validStick(xAxis2, yAxis2)) {    //DIAGONAL
                if (Math.abs(yAxis2) < xAxis2) {
                    rob.driveTrainMovement(ROTATION_SPEED, Rover.movements.cw);
                    //Rotate
                } else if (-Math.abs(yAxis2) > xAxis2) {
                    rob.driveTrainMovement(ROTATION_SPEED, Rover.movements.ccw);
                    //Rotate
                }

            } else {
                rob.stopDrivetrain();
            }


            if (gamepad1.y && !rob.latchingLimit.getState()){
                rob.anyMovement(0.8, Rover.movements.rackCompress, rob.rack);
            }
            else if (gamepad1.a && !rob.deployingLimit.getState()){
                rob.anyMovement(0.8, Rover.movements.rackExtend, rob.rack);
            }
            else {
                rob.rack.setPower(0);
            }

            if (gamepad2.a){
                rob.arm.setPower(0.3);
            }
            else if (gamepad2.y){
                rob.arm.setPower(-0.1);
            }
            else {
                rob.arm.setPower(0);
            }



            if (gamepad2.left_trigger > 0.25){
                rob.linear.setPower(0.9);
            }
            else if (gamepad2.right_trigger > 0.25){
                rob.linear.setPower(-0.9);
            }
            else {
                rob.linear.setPower(0);
            }

            if (gamepad2.dpad_up){
                rob.collector.setPower(0.7);
            }
            else if (gamepad2.dpad_down){
                rob.collector.setPower(-0.7);
            }
            else {
                rob.collector.setPower(0);
            }
        }
    }

}
