package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

import static org.firstinspires.ftc.teamcode.Control.Constants.ROTATION_SPEED;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorBLS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorBRS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorFLS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorFRS;


@TeleOp(name = "DriveMode Lil Bot", group = "Lil Bot")

public class DriveModeLilBot extends TeleOpControl {


    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode() throws InterruptedException{


        setRob(new Rover(hardwareMap, runtime, this, Rover.setupType.drive));


        DcMotor collector = hardwareMap.dcMotor.get("collector");
        DcMotor rightShooter = hardwareMap.dcMotor.get("rightShooter");
        DcMotor leftShooter = hardwareMap.dcMotor.get("leftShooter");

        setRuntime(runtime);
        waitForStart();

        while (opModeIsActive()) {

            // GAMEPAD OBJECT
            standardGamepadData();


            if (rightStickButtonPressed) {
                // CLOCKWISE
                
                rob.driveTrainMovement(ROTATION_SPEED, Rover.movements.cw);
            } else if (leftStickButtonPressed) {
                // COUNTERCLOCKWISE
                rob.driveTrainMovement(ROTATION_SPEED, Rover.movements.ccw);
            }
            else if (validStick(xAxis1, yAxis1)) { //MAIN DIRECTIONS

                if (yAxis1 >= Math.abs(xAxis1)) {
                    rob.driveTrainMovement(fb, Rover.movements.forward);
                    //FORWARD
                }else if (yAxis1 <= -Math.abs(xAxis1)) {
                    rob.driveTrainMovement(fb, Rover.movements.backward);
                    //BACKWARD
                } else if (Math.abs(yAxis1) < xAxis1) {
                    rob.driveTrainMovement(fb, Rover.movements.right);
                    //RIGHT
                } else if (-Math.abs(yAxis1) > xAxis1) {
                    rob.driveTrainMovement(fb, Rover.movements.left);
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



            //





            if (gamepad1.right_trigger>0.5){
                rightShooter.setPower(-0.95);
                leftShooter.setPower(0.95);
            }
            else if (gamepad1.left_trigger>0.5){
                rightShooter.setPower(0.95);
                leftShooter.setPower(-0.95);
            }
            else{
                rightShooter.setPower(0);
                leftShooter.setPower(0);

            }

            if (gamepad1.dpad_up){
                rightShooter.setPower(-0.5);
                leftShooter.setPower(0.5);
            }
            else if (gamepad1.dpad_down){
                rightShooter.setPower(0.5);
                leftShooter.setPower(-0.5);
            }
            else{
                rightShooter.setPower(0);
                leftShooter.setPower(0);

            }

            if (gamepad1.a) {
                collector.setPower(0.8);
                rightShooter.setPower(-0.95);
                leftShooter.setPower(0.95);

            }
            else if (gamepad1.y){
                collector.setPower(-0.8);
                rightShooter.setPower(0.95);
                leftShooter.setPower(-0.95);
            }
            else {
                collector.setPower(0);

            }


        }
    }

}
