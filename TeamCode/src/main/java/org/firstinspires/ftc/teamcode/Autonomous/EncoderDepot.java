package org.firstinspires.ftc.teamcode.Autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

@Autonomous(name="EncoderDepot", group ="Smart")

public class EncoderDepot extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();
    private double rotationperinch=0.7;
    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Rover.setupType.drive, Rover.setupType.latching, Rover.setupType.vuforia, Rover.setupType.phoneswivel);

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
                    rob.turn(35,Rover.turnside.ccw,1,Rover.axis.center);
                    rob.driveTrainEncoderMovement(0.5,53*rotationperinch,5,200,Rover.movements.forward);

                    rob.turn(65,Rover.turnside.cw,1,Rover.axis.center);
                    rob.driveTrainEncoderMovement(0.5,28.3*rotationperinch,5,200,Rover.movements.forward);

                    rob.arm.setPower(0.3);
                    sleep(500);
                    rob.arm.setPower(0);

                    rob.driveTrainEncoderMovement(0.75,116*rotationperinch,5,200,Rover.movements.backward);

                    rob.turn(180,Rover.turnside.cw,1,Rover.axis.center);
                    rob.linear.setPower(1);
                    sleep(500);
                    rob.linear.setPower(0);
                case CENTER:
                case UNKNOWN:
                    rob.driveTrainEncoderMovement(0.5,73.2*rotationperinch,5,200,Rover.movements.forward);

                    rob.arm.setPower(0.3);
                    sleep(500);
                    rob.arm.setPower(0);

                    rob.turn(45,Rover.turnside.cw,1,Rover.axis.center);
                    rob.driveTrainEncoderMovement(0.75,117.8*rotationperinch,5,200,Rover.movements.forward);

                    rob.linear.setPower(1);
                    sleep(500);
                    rob.linear.setPower(0);

                case RIGHT:
                    rob.turn(35,Rover.turnside.cw,1,Rover.axis.center);
                    rob.driveTrainEncoderMovement(0.5,53.1*rotationperinch,5,200,Rover.movements.forward);

                    rob.turn(55,Rover.turnside.ccw,1,Rover.axis.center);
                    rob.driveTrainEncoderMovement(0.5,35.4*rotationperinch,5,200,Rover.movements.forward);

                    rob.arm.setPower(0.3);
                    sleep(500);
                    rob.arm.setPower(0);

                    rob.turn(70,Rover.turnside.cw,1,Rover.axis.center);
                    rob.driveTrainEncoderMovement(0.75,116.8*rotationperinch,5,200,Rover.movements.backward);
                    rob.turn(180,Rover.turnside.cw,1,Rover.axis.center);
                    rob.linear.setPower(1);
                    sleep(500);
                    rob.linear.setPower(0);
            }
        }
    }
}


