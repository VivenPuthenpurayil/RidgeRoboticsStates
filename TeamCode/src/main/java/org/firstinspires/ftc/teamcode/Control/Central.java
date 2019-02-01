package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Central extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();

    public Rover rob;

    public Rover.movements[] allMovements = {Rover.movements.forward, Rover.movements.backward, Rover.movements.right, Rover.movements.left, Rover.movements.tr, Rover.movements.bl, Rover.movements.tl, Rover.movements.br,Rover.movements.cw, Rover.movements.ccw};

    public void setRob(Rover rob) {
        this.rob = rob;
    }

    public void setup(ElapsedTime rtime, Rover.setupType... setup) throws InterruptedException {
        this.setRob(new Rover(hardwareMap, runtime, this, setup));
        setRuntime(rtime);
        this.waitForStart();
        this.runtime.reset();
        if (rob.vuforiaMode){
            rob.vuforia.targetsRoverRuckus.activate();
        }
        if (rob.tensorflowMode){
            rob.vuforia.tfod.activate();
        }

    }

    public void setRuntime(ElapsedTime runtime) {
        this.runtime = runtime;
    }
}
