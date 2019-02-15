package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

import static org.firstinspires.ftc.teamcode.Control.Constants.ROTATION_SPEED;


@TeleOp(name = "Testbot", group = "Smart")

public class testbottest extends TeleOpControl {


    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        setup(runtime, Rover.setupType.drive);


        while (opModeIsActive()) {

            // GAMEPAD OBJECT
            standardGamepadData();


            if (rightStickButtonPressed) {
                // CLOCKWISE
                rob.driveTrainMovement(ROTATION_SPEED, Rover.movements.cw);
            } else if (leftStickButtonPressed) {
                // COUNTERCLOCKWISE
                rob.driveTrainMovement(ROTATION_SPEED, Rover.movements.ccw);
            } else if (gamepad1.dpad_up) {
                rob.driveTrainMovement(0.2, Rover.movements.forward);
            } else if (gamepad1.dpad_right) {
                rob.driveTrainMovement(0.2, Rover.movements.right);
            } else if (gamepad1.dpad_left) {
                rob.driveTrainMovement(0.2, Rover.movements.left);
            } else if (gamepad1.dpad_down) {

                rob.driveTrainMovement(0.2, Rover.movements.backward);
            } else if (validStick(xAxis1, yAxis1)) { //MAIN DIRECTIONS

                if (yAxis1 >= Math.abs(xAxis1)) {
                    rob.driveTrainMovement(fb, Rover.movements.forward);
                    //FORWARD
                } else if (yAxis1 <= -Math.abs(xAxis1)) {
                    rob.driveTrainMovement(fb, Rover.movements.backward);
                    //BACKWARD
                } else if (Math.abs(yAxis1) < xAxis1) {
                    rob.driveTrainMovement(rl, Rover.movements.right);
                    //RIGHT
                } else if (-Math.abs(yAxis1) > xAxis1) {
                    rob.driveTrainMovement(rl, Rover.movements.left);
                    //LEFT
                }
            } else if (validStick(xAxis2, yAxis2)) {    //DIAGONAL

                if (yAxis2 >= 0 && xAxis2 >= 0) {
                    rob.driveTrainMovement(diagonalSpeed, Rover.movements.tr);
                    //TOP RIGHT
                } else if (yAxis2 >= 0 && xAxis2 < 0) {
                    rob.driveTrainMovement(diagonalSpeed, Rover.movements.tl);
                    //TOP LEFT
                } else if (yAxis2 < 0 && xAxis2 >= 0) {
                    rob.driveTrainMovement(diagonalSpeed, Rover.movements.br);
                    //BOTTOM RIGHT
                } else if (yAxis2 < 0 && xAxis2 < 0) {
                    rob.driveTrainMovement(diagonalSpeed, Rover.movements.bl);
                    //BOTTOM LEFT
                }
            } else {
                rob.stopDrivetrain();
            }

            if(gamepad1.right_trigger > 0.25){



            }

        }
    }
}