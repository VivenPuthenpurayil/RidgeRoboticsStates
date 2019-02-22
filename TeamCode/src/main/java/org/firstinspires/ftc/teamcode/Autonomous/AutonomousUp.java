package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name = "AutonomusUp", group = "Autonomous")
public class AutonomousUp extends LinearOpMode {

    float WD = 6;
    float epc = (float) (360 / (WD * 3.141592653589));
    private DcMotor LeftBackDrive;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor RightBackDrive;
    private DcMotor LeftFrontDrive;
    private DcMotor RightFrontDrive;
    private DcMotor Lifter;

    public void GoForward (){
        LeftFrontDrive.setPower(1);
        LeftBackDrive.setPower(1);
        RightBackDrive.setPower(1);
        RightFrontDrive.setPower(1);
    }
    public void GoBackward (){
        LeftFrontDrive.setPower(-1);
        LeftBackDrive.setPower(-1);
        RightBackDrive.setPower(-1);
        RightFrontDrive.setPower(-1);
    }
    public void GoLeft (){
        LeftFrontDrive.setPower(1);
        LeftBackDrive.setPower(1);
        RightBackDrive.setPower(1);
        RightFrontDrive.setPower(-1);
    }
    public void GoRight (){
        LeftFrontDrive.setPower(1);
        LeftBackDrive.setPower(-1);
        RightBackDrive.setPower(1);
        RightFrontDrive.setPower(-1);
    }
    public void StopMotors (){
        LeftFrontDrive.setPower(0);
        LeftBackDrive.setPower(0);
        RightBackDrive.setPower(0);
        RightFrontDrive.setPower(0);
    }


    public void runOpMode() throws InterruptedException {

        LeftFrontDrive = hardwareMap.dcMotor.get("LeftFrontDrive");
        RightFrontDrive = hardwareMap.dcMotor.get("RightFrontDrive");
        LeftBackDrive = hardwareMap.dcMotor.get("LeftBackDrive");
        RightBackDrive = hardwareMap.dcMotor.get("RightBackDrive");
        Lifter = hardwareMap.dcMotor.get("Lifter");
        LeftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        LeftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        runtime.reset();

        Lifter.setPower(1);
        sleep(5000);
        Lifter.setPower(0);
        LeftFrontDrive.setPower(-1);
        LeftBackDrive.setPower(1);
        RightFrontDrive.setPower(1);
        RightBackDrive.setPower(1);
        sleep(3000);
        StopMotors();
        GoForward();
        sleep(13000);
    }
}