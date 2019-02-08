package org.firstinspires.ftc.teamcode.Tests.System;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.Rover;
import org.firstinspires.ftc.teamcode.Control.TeleOpControl;

@TeleOp(name = "Collection Test", group = "Test")
public class Test_Collection extends TeleOpControl {
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motorRS = hardwareMap.dcMotor.get("rightShooter");
        DcMotor motorLS = hardwareMap.dcMotor.get("leftShooter");
        DcMotor motorSpinner = hardwareMap.dcMotor.get("spinner");
        DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
        DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");
        DcMotor motorFL = hardwareMap.dcMotor.get("motorFL");
        DcMotor motorBL = hardwareMap.dcMotor.get("motorBL");
        motorRS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        setRob(new Rover(hardwareMap, runtime, this));
        setRuntime(runtime);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            standardGamepadData();

            if (gamepad1.a){ // motor test
                motorRS.setPower(1);
                motorLS.setPower(1);
                motorSpinner.setPower(-1);
                motorFR.setPower(1);
                motorBR.setPower(1);
                motorFL.setPower(1);
                motorBL.setPower(1);
            }
            else if (gamepad1.y) { //encoder test
                motorRS.setPower(-1);
                motorLS.setPower(-1);
                motorSpinner.setPower(1);

            }else {
                motorRS.setPower(0);
                motorLS.setPower(0);
                motorSpinner.setPower(0);
                motorFR.setPower(1);
                motorBR.setPower(1);
                motorFL.setPower(1);
                motorBL.setPower(1);
            }
            }
        }

    public void encoder(DcMotor motor) throws InterruptedException {



    }
}
