package org.firstinspires.ftc.teamcode.Tests.Functionality;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.Test;


@Autonomous(name = "Ultrasonic Test", group = "Test")

public class Test_Ultrasonic extends Test {

    public ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException{
        ModernRoboticsI2cRangeSensor FRU = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "FRU");
        ModernRoboticsI2cRangeSensor FLU = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "FLU");
        ModernRoboticsI2cRangeSensor BLU = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "BLU");
        ModernRoboticsI2cRangeSensor BRU = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "BRU");

        telemetry.addData("FRU distance", FRU.status());
        telemetry.addData("FRU distance", FRU.getDistance(DistanceUnit.CM));
        telemetry.addData("FLU distance", FLU.getDistance(DistanceUnit.CM));
        telemetry.addData("BLU distance", BLU.getDistance(DistanceUnit.CM));
        telemetry.addData("BRU distance", BRU.getDistance(DistanceUnit.CM));
        telemetry.update();

        setRuntime(runtime);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            telemetry.addData("FRU distance", FRU.getDistance(DistanceUnit.CM));
            telemetry.addData("FLU distance", FLU.getDistance(DistanceUnit.CM));
            telemetry.addData("BLU distance", BLU.getDistance(DistanceUnit.CM));
            telemetry.addData("BRU distance", BRU.getDistance(DistanceUnit.CM));

            telemetry.update();
            sleep(30);
        }
    }




}
