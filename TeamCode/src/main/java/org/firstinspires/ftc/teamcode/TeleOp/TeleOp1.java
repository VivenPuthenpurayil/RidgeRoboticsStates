package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import  com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp")
public class TeleOp1 extends LinearOpMode {
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
    // private DcMotor StorageArm;
    private DcMotor Lifter;

    @Override
    public void runOpMode() throws InterruptedException {
        // LeftDrive = hardwareMap.dcMotor.get("LeftDrive");
        //   RightDrive = hardwareMap.dcMotor.get("RightDrive");
        motorFL = hardwareMap.dcMotor.get("LeftFrontDrive");
        motorFR = hardwareMap.dcMotor.get("RightFrontDrive");
        motorBL = hardwareMap.dcMotor.get("LeftBackDrive");
        motorBR = hardwareMap.dcMotor.get("RightBackDrive");
        //StorageArm = hardwareMap.dcMotor.get("SA");
        double storagepower = 0.7;
        Lifter = hardwareMap.dcMotor.get("Lifter");


        waitForStart();

        while (opModeIsActive()) {
            // throttle:  left_stick_y ranges from -1 to 1, where -1 is full up,  and 1 is full down
            // direction: left_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right
            float fb  = -gamepad1.left_stick_y;
            float rl =  gamepad1.left_stick_x;
            float y2 = -gamepad1.right_stick_y;
            float x2 = gamepad1.right_stick_x;

            boolean rs = gamepad1.right_stick_button;
            boolean ls = gamepad1.left_stick_button;
            boolean bPrevState = false;
            boolean bCurrState = false;
            boolean x = true;
            int s = 1;
            double p = Math.sqrt((y2*y2) + (x2*x2));
            // clip the right/left values so that the values never exceed +/- 1
            int count = 0;
            int counter = 0;
            fb = Range.clip(fb, -1, 1);
            rl  = Range.clip(rl,  -1, 1);
            y2 = Range.clip(y2, -1, 1);
            x2 = Range.clip(x2, -1, 1);
            // write the values to the motors

            bCurrState = gamepad1.right_bumper;

            // check for button state transitions.
            if ((bCurrState == true) && (bCurrState != bPrevState))  {

                // button is transitioning to a pressed state. So Toggle LED
                x = !x;
            }

            // update previous state variable.
            bPrevState = bCurrState;


            if(x) {
                if (fb > rl && fb > -rl && fb > 0.1 & !ls) {
                    //Forward
                    motorFR.setPower(fb);
                    motorBR.setPower(fb);
                    motorFL.setPower(-fb);
                    motorBL.setPower(-fb);
                } else if (fb < rl && fb < -rl && fb < -0.1 & !ls) {
                    //Backward
                    motorFR.setPower(fb);
                    motorBR.setPower(fb);
                    motorFL.setPower(-fb);
                    motorBL.setPower(-fb);
                } else if (fb > rl && fb < -rl && rl < -0.1 & !ls) {
                    //Left
                    motorFR.setPower(-rl);
                    motorBR.setPower(rl);
                    motorFL.setPower(-rl);
                    motorBL.setPower(rl);
                } else if (fb < rl && fb > -rl && rl > 0.1 & !ls) {
                    //Right
                    motorFR.setPower(-rl);
                    motorBR.setPower(rl);
                    motorFL.setPower(-rl);
                    motorBL.setPower(rl);
                } else if (gamepad1.left_stick_button) {
                    //Spin CCW
                    motorFR.setPower(0.4);
                    motorBR.setPower(0.4);
                    motorFL.setPower(0.4);
                    motorBL.setPower(0.4);

                } else if (gamepad1.right_stick_button) {
                    //Spin CW
                    motorFR.setPower(-0.4);
                    motorBR.setPower(-0.4);
                    motorFL.setPower(-0.4);
                    motorBL.setPower(-0.4);

                } else if (y2 > 0.1 && x2 > 0.1 & !rs) {
                    //DIAGONAL TR
                    motorFR.setPower(0);
                    motorBR.setPower(p);
                    motorFL.setPower(-p);
                    motorBL.setPower(0);

                } else if (y2 > 0.1 && x2 < -0.1 & !rs) {
                    //DIAGONAL TL
                    motorFR.setPower(p);
                    motorBR.setPower(0);
                    motorFL.setPower(0);
                    motorBL.setPower(-p);

                } else if (y2 < -0.1 && x2 > 0.1 & !rs) {
                    //DIAGONAL BR
                    motorFR.setPower(-p);
                    motorBR.setPower(0);
                    motorFL.setPower(0);
                    motorBL.setPower(p);

                } else if (y2 < -0.1 && x2 < -0.1 & !rs) {
                    //DIAGONAL BL
                    motorFR.setPower(0);
                    motorBR.setPower(-p);
                    motorFL.setPower(p);
                    motorBL.setPower(0);

                }else if(gamepad1.dpad_up){
                    motorFR.setPower(0.5);
                    motorBR.setPower(-0.5);
                    motorFL.setPower(0.5);
                    motorBL.setPower(-0.5);

                }else if(gamepad1.dpad_down){
                    motorFR.setPower(-0.5);
                    motorBR.setPower(0.5);
                    motorFL.setPower(-0.5);
                    motorBL.setPower(0.5);

                }else if(gamepad1.dpad_left){
                    motorFR.setPower(-0.5);
                    motorBR.setPower(-0.5);
                    motorFL.setPower(0.5);
                    motorBL.setPower(0.5);

                }else if(gamepad1.dpad_right){
                    motorFR.setPower(0.5);
                    motorBR.setPower(0.5);
                    motorFL.setPower(-0.5);
                    motorBL.setPower(-0.5);

                }else {
                    motorFR.setPower(0);
                    motorBR.setPower(0);
                    motorFL.setPower(0);
                    motorBL.setPower(0);
                }
            }
            else {
                if (fb > rl && fb > -rl && fb > 0.1 & !ls) {
                    //Forward
                    motorFR.setPower(0.2);
                    motorBR.setPower(0.2);
                    motorFL.setPower(-0.2);
                    motorBL.setPower(-0.2);
                    telemetry.addLine("Should be working");
                    telemetry.update();
                } else if (fb < rl && fb < -rl && fb < -0.1 & !ls) {
                    //Backward
                    motorFR.setPower(-0.2);
                    motorBR.setPower(-0.2);
                    motorFL.setPower(0.2);
                    motorBL.setPower(0.2);
                    telemetry.addLine("Should be working");
                    telemetry.update();
                } else if (fb > rl && fb < -rl && rl < -0.1 & !ls) {
                    //Left
                    motorFR.setPower(0.2);
                    motorBR.setPower(-0.2);
                    motorFL.setPower(0.2);
                    motorBL.setPower(-0.2);
                    telemetry.addLine("Should be working");
                    telemetry.update();
                } else if (fb < rl && fb > -rl && rl > 0.1 & !ls) {
                    //Right
                    motorFR.setPower(-0.2);
                    motorBR.setPower(0.2);
                    motorFL.setPower(-0.2);
                    motorBL.setPower(0.2);
                    telemetry.addLine("Should be working");
                    telemetry.update();
                }else if(gamepad1.dpad_up){
                    motorFR.setPower(0.2);
                    motorBR.setPower(-0.2);
                    motorFL.setPower(0.2);
                    motorBL.setPower(-0.2);

                }else if(gamepad1.dpad_down){
                    motorFR.setPower(-0.2);
                    motorBR.setPower(0.2);
                    motorFL.setPower(-0.2);
                    motorBL.setPower(0.2);

                }else if(gamepad1.dpad_left){
                    motorFR.setPower(-0.2);
                    motorBR.setPower(-0.2);
                    motorFL.setPower(0.2);
                    motorBL.setPower(0.2);

                }else if(gamepad1.dpad_right){
                    motorFR.setPower(0.2);
                    motorBR.setPower(0.2);
                    motorFL.setPower(-0.2);
                    motorBL.setPower(-0.2);

                }
                else if (gamepad1.left_stick_button) {
                    //Spin CCW
                    motorFR.setPower(0.2);
                    motorBR.setPower(0.2);
                    motorFL.setPower(0.2);
                    motorBL.setPower(0.2);
                    telemetry.addLine("Should be working");
                    telemetry.update();


                } else if (gamepad1.right_stick_button) {
                    //Spin CW
                    motorFR.setPower(-0.2);
                    motorBR.setPower(-0.2);
                    motorFL.setPower(-0.2);
                    motorBL.setPower(-0.2);
                    telemetry.addLine("Should be working");
                    telemetry.update();

                } else if (y2 > 0.1 && x2 > 0.1 & !rs) {
                    //DIAGONAL TR
                    motorFR.setPower(0);
                    motorBR.setPower(0.2);
                    motorFL.setPower(-0.2);
                    motorBL.setPower(0);
                    telemetry.addLine("Should be working");
                    telemetry.update();

                } else if (y2 > 0.1 && x2 < -0.1 & !rs) {
                    //DIAGONAL TL
                    motorFR.setPower(0.2);
                    motorBR.setPower(0);
                    motorFL.setPower(0);
                    motorBL.setPower(-0.2);
                    telemetry.addLine("Should be working");
                    telemetry.update();

                } else if (y2 < -0.1 && x2 > 0.1 & !rs) {
                    //DIAGONAL BR
                    motorFR.setPower(-0.2);
                    motorBR.setPower(0);
                    motorFL.setPower(0);
                    motorBL.setPower(0.2);
                    telemetry.addLine("Should be working");
                    telemetry.update();

                } else if (y2 < -0.1 && x2 < -0.1 & !rs) {
                    //DIAGONAL BL
                    motorFR.setPower(0);
                    motorBR.setPower(-0.2);
                    motorFL.setPower(0.2);
                    motorBL.setPower(0);
                    telemetry.addLine("Should be working");
                    telemetry.update();

                } else {
                    motorFR.setPower(0);
                    motorBR.setPower(0);
                    motorFL.setPower(0);
                    motorBL.setPower(0);
                }
            }
            telemetry.addData("CRAWL MODE: ", gamepad1.right_bumper);
            telemetry.update();
//--------------------- ----------------------------------------------

            if(gamepad1.y){
                Lifter.setPower(storagepower);
            }
            else{
                Lifter.setPower(0);
            }
            if(gamepad1.b){

                Lifter.setPower(-storagepower);
            }
            else{
                Lifter.setPower(0);
            }


        }
    }

}