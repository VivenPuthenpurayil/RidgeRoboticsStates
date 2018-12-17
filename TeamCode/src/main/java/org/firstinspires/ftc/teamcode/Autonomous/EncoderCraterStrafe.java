package org.firstinspires.ftc.teamcode.Autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

@Autonomous(name="EncoderCrater Strafe", group ="Smart")


public class EncoderCraterStrafe extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();
    private double rotationperinch=0.7;
    private double strafeRPI=1.4;
    private double rotationperdegree = 0.06;

    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Rover.setupType.autonomous);

        while (opModeIsActive()){
            rob.deploy();
            SamplingOrderDetector detector = new SamplingOrderDetector();
            detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            detector.useDefaults();
            detector.downscale = 0.4;
            detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
            detector.maxAreaScorer.weight = 0.005;
            detector.ratioScorer.weight = 5;
            detector.ratioScorer.perfectRatio = 1.0;
            detector.enable();
            switch(detector.getCurrentOrder()){
                case LEFT:
                case UNKNOWN:

                case CENTER:


                case RIGHT:
                    rob.driveTrainEncoderMovement(0.5,36*rotationperinch,10,200,Rover.movements.forward);
                    rob.driveTrainEncoderMovement(0.5,18*rotationperinch,10,200,Rover.movements.backward);
                    rob.driveTrainEncoderMovement(0.5,24*strafeRPI,10,200,Rover.movements.right);
                    rob.driveTrainEncoderMovement(0.5, 45*rotationperdegree,10,200,Rover.movements.ccw);
                    rob.driveTrainEncoderMovement(0.5,120*rotationperinch,10,200,Rover.movements.backward);
                    rob.driveTrainEncoderMovement(1,130*rotationperinch,10,200,Rover.movements.forward);
                    rob.linear.setPower(0.3);
                    sleep(2000);
                    rob.linear.setPower(0);

            }
        }
    }

}
