package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Central extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();

    public Rover rob;


    public void setRob(Rover rob) {
        this.rob = rob;
    }

    public void setup(ElapsedTime rtime, Rover.setupType... setup) throws InterruptedException {
        setRob(new Rover(hardwareMap, runtime, this, setup));
        setRuntime(rtime);
        this.waitForStart();
        this.runtime.reset();
        if (rob.vuforiaMode){
            rob.vuforia.targetsRoverRuckus.activate();
        }

    }

    public void setRuntime(ElapsedTime runtime) {
        this.runtime = runtime;
    }
}
