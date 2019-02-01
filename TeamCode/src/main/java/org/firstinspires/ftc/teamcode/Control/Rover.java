package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Mat;

import java.util.Arrays;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_MOTOR_NEVEREST;
import static org.firstinspires.ftc.teamcode.Control.Constants.armS;
import static org.firstinspires.ftc.teamcode.Control.Constants.collectorS;
import static org.firstinspires.ftc.teamcode.Control.Constants.deployingLimitS;
import static org.firstinspires.ftc.teamcode.Control.Constants.imuS;
import static org.firstinspires.ftc.teamcode.Control.Constants.latchingLimitS;
import static org.firstinspires.ftc.teamcode.Control.Constants.linearS;
import static org.firstinspires.ftc.teamcode.Control.Constants.mmPerInch;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorBLS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorBRS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorFLS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorFRS;
import static org.firstinspires.ftc.teamcode.Control.Constants.phoneSwivelS;
import static org.firstinspires.ftc.teamcode.Control.Constants.rackS;
import static org.firstinspires.ftc.teamcode.Control.Rover.movements.backward;
import static org.firstinspires.ftc.teamcode.Control.Rover.movements.cw;
import static org.firstinspires.ftc.teamcode.Control.Rover.movements.forward;
import static org.firstinspires.ftc.teamcode.Control.Rover.movements.forward2;
import static org.firstinspires.ftc.teamcode.Control.Rover.movements.left;
import static org.firstinspires.ftc.teamcode.Control.Rover.movements.right;

public class Rover {

    public Rover(HardwareMap hardwareMap, ElapsedTime runtime, Central central, setupType... setup) throws InterruptedException {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.central = central;

        StringBuilder i = new StringBuilder();
        for (setupType type : setup) {
            switch (type){
                case drive:
                    setupDrivetrain();
                    break;

                case latching:
                    setupLatching();
                    break;

                case latchingTele:
                    setupLatchingTele();
                    break;

                case mineralControl:
                    setupMineralControl();
                    break;

                case imu:
                    setupIMU();
                    break;

                case vuforia:
                    setupVuforia(0);
                    break;

                case tensorflow:
                    setupVuforia(1);
                    break;
                case sensors:
                    setupSensors();
                    break;


                case autonomous:
                    setupLatching();
                    setupIMU();
                    setupDrivetrain();
                    setupMineralControl();
                    setupVuforia(1);
                    //setupPhone();
                    //setupSensors();
                    break;


            }
            i.append(type.name()).append(" ");
        }
        central.telemetry.addLine(i.toString());
        central.telemetry.update();


    }

    public void setupLatchingTele() throws InterruptedException {
        rack = motor(rackS, DcMotorSimple.Direction.FORWARD);

        deployingLimit = hardwareMap.digitalChannel.get(deployingLimitS);//name it limit in config pls <3
        latchingLimit = hardwareMap.digitalChannel.get(latchingLimitS);

        encoder(EncoderMode.ON, rack);

    }

    public Rover(HardwareMap hardwareMap, ElapsedTime runtime, Central central) throws InterruptedException {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.central = central;

    }

    // important non-configuration fields
    public ElapsedTime runtime;     //set in constructor to the runtime of running class
    public Central central;         //set in constructor to the runtime of running class
    public HardwareMap hardwareMap; //set in constructor to the runtime of running class


    //----specfic non-configuration fields
    //none rn



    //----------------CONFIGURATION FIELDS--------------------


    SamplingOrder sampleStatus = SamplingOrder.UNCHECKED;





    //----  MAPPING         ----
    ModernRoboticsI2cRangeSensor rangeSensorfront;
    ModernRoboticsI2cRangeSensor rangeSensorback;
    ModernRoboticsI2cRangeSensor rangeSensorright;
    ModernRoboticsI2cRangeSensor rangeSensorleft;


    //----  DRIVE           ----
    public  DcMotor[] drivetrain;   //set in motorDriveMode() for drivetrain movement functions

    public DcMotor motorFR;
    public DcMotor motorFL;
    public DcMotor motorBR;
    public DcMotor motorBL;

    public double StrafetoTotalPower = 2/3;
    public double mecanumAngle = 36; //from forwards, in degrees
    public double communism = StrafetoTotalPower*Math.cos(Math.toRadians(mecanumAngle*2));

    //----  MINERAL CONTROL ----

    public DcMotor arm;
    public DcMotor linear;
    public DcMotor collector;

    //----  LATCHING SYSTEM ----
    public DcMotor rack;

    public DigitalChannel latchingLimit;
    public DigitalChannel deployingLimit;

    //----       IMU        ----

    public BNO055IMUImpl imu;
    public BNO055IMUImpl.Parameters parameters = new BNO055IMUImpl.Parameters();
    public Orientation current;
    public static boolean isnotstopped;

    //----  PHONE SWIVEL    ----
    public Servo phoneSwivel;

    //---- VUFORIA HANDLER  ----
    public VuforiaHandler vuforia;
    public boolean vuforiaMode;
    public boolean tensorflowMode;
    public float initorient;




    //-----         LATCHING FUNCTIONS          --------------
    public void latchInit() throws InterruptedException{
        while(!latchingLimit.getState())
        {
            anyMovement(0.8, Rover.movements.rackCompress, rack);
        }
        rack.setPower(0);
        central.telemetry.addLine("Latched");
        central.telemetry.update();
    }

    public void latch() throws InterruptedException{
        while(!latchingLimit.getState() && central.opModeIsActive())
        {
            anyMovement(0.8, Rover.movements.rackCompress, rack);
        }
        rack.setPower(0);
        central.telemetry.addLine("Latched");
        central.telemetry.update();
    }
    public void deploy() throws InterruptedException{

        while(!deployingLimit.getState() && central.opModeIsActive())
        {
            anyMovement(0.8, movements.rackExtend, rack);
            sample();
        }
        rack.setPower(0);

        if (sampleStatus==SamplingOrder.UNKNOWN){
            sample();
        }
        //driveTrainEncoderMovement(0.8, 0.5, 3, 50, cw);
        //while(Math.absimu.getAcceleration())
        driveTrainTimeMovement(0.8, backward, 500, 50);
        driveTrainTimeMovement(0.8, right, 500, 50);
        driveTrainTimeMovement(0.8, forward, 500, 50);
        //driveTrainEncoderMovement(0.8, 5, 3, 50, ccw);
        //driveTrainEncoderMovement(0.8, 2, 3, 50, backward);

    }

    //------        MAPPING FUNCTIONS           --------------

    public double rangeDistancefront(){
        return rangeSensorfront.getDistance(DistanceUnit.CM);
    }
    public double rangeDistanceback(){
        return rangeSensorback.getDistance(DistanceUnit.CM);
    }
    public double rangeDistanceright(){
        return rangeSensorright.getDistance(DistanceUnit.CM);
    }
    public double rangeDistanceleft(){
        return rangeSensorleft.getDistance(DistanceUnit.CM);
    }


    //----          SETUP FUNCTIONS             --------------

    public void setupPhone() throws InterruptedException {
        phoneSwivel = servo(phoneSwivelS, Servo.Direction.FORWARD, 0, 1, 0);

    }

    public void setupSensors() {

        rangeSensorfront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "frontRange");
        rangeSensorback = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "backRange");

        rangeSensorright = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRange");
        rangeSensorleft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftRange");
    }

    public void setupVuforia(int i) {
        if (i == 1) {
            vuforia = new VuforiaHandler(central, VuforiaHandler.type.both);
            vuforiaMode = true;
            tensorflowMode = true;
        }
        else if (i == 0){
            vuforia = new VuforiaHandler(central, VuforiaHandler.type.images);
            vuforiaMode = true;
        }

    }

    public void setupDrivetrain() throws InterruptedException {
        motorFR = motor(motorFRS, DcMotorSimple.Direction.FORWARD);
        motorFL = motor(motorFLS, DcMotorSimple.Direction.FORWARD);
        motorBR = motor(motorBRS, DcMotorSimple.Direction.FORWARD);
        motorBL = motor(motorBLS, DcMotorSimple.Direction.FORWARD);

        motorDriveMode(EncoderMode.ON, motorFR, motorFL, motorBR, motorBL);
    }
    public void setupLatching() throws InterruptedException {
        rack = motor(rackS, DcMotorSimple.Direction.FORWARD);

        deployingLimit = hardwareMap.digitalChannel.get(deployingLimitS);//name it limit in config pls <3
        latchingLimit = hardwareMap.digitalChannel.get(latchingLimitS);

        encoder(EncoderMode.ON, rack);

        latchInit();
    }

    public void setupMineralControl() throws InterruptedException{
        arm = motor(armS, DcMotorSimple.Direction.FORWARD);
        linear = motor(linearS, DcMotorSimple.Direction.FORWARD);
        collector = motor(collectorS, DcMotorSimple.Direction.FORWARD);


        encoder(EncoderMode.ON, arm, linear);

    }

    public void setupMarker() throws InterruptedException{

    }

    public void setupIMU() throws InterruptedException{
        parameters.angleUnit = BNO055IMUImpl.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMUImpl.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true; //copypasted from BNO055IMU sample code, no clue what this does
        parameters.loggingTag = "imu"; //copypasted from BNO055IMU sample code, no clue what this does
        imu = hardwareMap.get(BNO055IMUImpl.class, imuS);
        imu.initialize(parameters);
        initorient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        central.telemetry.addData("IMU status", imu.getSystemStatus());
        central.telemetry.update();


    }

    //-----------------------HARDWARE SETUP FUNCTIONS---------------------------------------
    public DcMotor motor(String name, DcMotor.Direction direction) throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);
        return motor;
    }
    public Servo servo(String name, Servo.Direction direction, double min, double max, double start) throws InterruptedException {
        Servo servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
        servo.scaleRange(min, max);
        servo.setPosition(start);
        return servo;
    }
    public CRServo servo(String name, DcMotorSimple.Direction direction, double startSpeed) throws InterruptedException {
        CRServo servo = hardwareMap.crservo.get(name);
        servo.setDirection(direction);

        servo.setPower(0);
        return servo;
    }
    public ColorSensor colorSensor(String name, boolean ledOn) throws InterruptedException {
        ColorSensor sensor = hardwareMap.colorSensor.get(name);
        sensor.enableLed(ledOn);

        central.telemetry.addData("Beacon Red Value: ", sensor.red());
        central.telemetry.update();

        return sensor;
    }
    public ModernRoboticsI2cRangeSensor ultrasonicSensor(String name) throws InterruptedException {

        return hardwareMap.get(ModernRoboticsI2cRangeSensor.class, name);
    }
    public void encoder(EncoderMode mode, DcMotor... motor) throws InterruptedException {
        switch (mode) {
            case ON:
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                central.idle();
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                break;
            case OFF:
                break;
        }

    }

    public void motorDriveMode(EncoderMode mode, DcMotor... motor) throws InterruptedException {
        switch (mode) {
            case ON:
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                central.idle();
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                break;
            case OFF:
                break;
        }

        this.drivetrain = motor;

    }

    //------------------------ENCODER MOVEMENTS----------------------------

    public void driveTrainEncoderMovement(double speed, double distance, double timeoutS, long waitAfter, movements movement) throws  InterruptedException{

        int[] targets = new int[drivetrain.length];
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (central.opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            for (DcMotor motor : drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * distance * COUNTS_PER_MOTOR_NEVEREST);
            }
            for (DcMotor motor: drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:drivetrain){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (central.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                central.idle();
                for (DcMotor motor: drivetrain){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            // Stop all motion;
            for (DcMotor motor: drivetrain){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            central.sleep(waitAfter);


        }
    }
    public void encoderMovement(double speed, double distance, double timeoutS, long waitAfter, movements movement, DcMotor... motors) throws  InterruptedException{

        int[] targets = new int[motors.length];
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (central.opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            for (DcMotor motor : motors){
                int x = Arrays.asList(motors).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * distance * 4.0 * 3.14165 * COUNTS_PER_INCH);
            }
            for (DcMotor motor: motors){
                int x = Arrays.asList(motors).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:motors){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (central.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                central.idle();
                for (DcMotor motor: motors){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            // Stop all motion;
            for (DcMotor motor: motors){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            central.sleep(waitAfter);


        }
    }

    public void turn(float target, turnside direction, double speed, axis rotation_Axis) throws InterruptedException{

        central.telemetry.addData("IMU State: ", imu.getSystemStatus());
        central.telemetry.update();

        double start = getDirection();
        double end = start + ((direction == turnside.cw) ? target : -target);
        isnotstopped = true;
        try {
            switch (rotation_Axis) {
                case center:
                    driveTrainMovement(speed, (direction == turnside.cw) ? cw : movements.ccw);
                    break;
                case back:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwback : movements.ccwback);
                    break;
                case front:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwfront : movements.ccwfront);
                    break;
            }
        } catch (InterruptedException e) {
            isnotstopped = false;
        }
        while (!((end <= getDirection()+1) && end > getDirection() - 1) && central.opModeIsActive() && isnotstopped) {}
        try {
            stopDrivetrain();
        } catch (InterruptedException e) {
        }

    }

    public void turn2Wheel(float target, turnside direction, double speed) throws InterruptedException {
        turn(target, direction, speed, axis.back);
    }


    //------------------DRIVETRAIN TELEOP FUNCTIONS------------------------------------------------------------------------
    public void driveTrainMovement(double speed, movements movement) throws InterruptedException{
        double[] signs = movement.getDirections();
        for (DcMotor motor: drivetrain){
            int x = Arrays.asList(drivetrain).indexOf(motor);
            motor.setPower(signs[x]* speed);

        }
    }
    public void driveTrainMovement(double... speed) throws InterruptedException{

        for (int i = 0; i < drivetrain.length; i++) {
            drivetrain[i].setPower(speed[i]);
        }
    }
    public void driveTrainTimeMovement(double speed, movements movement, long duration, long waitAfter) throws InterruptedException{
        double[] signs = movement.getDirections();
        for (DcMotor motor: drivetrain){
            int x = Arrays.asList(drivetrain).indexOf(motor);
            motor.setPower(signs[x]* speed);

        }
        central.sleep(duration);
        stopDrivetrain();
        central.sleep(waitAfter);
    }

    public void anyMovement(double speed, movements movement, DcMotor... motors) throws InterruptedException{
        double[] signs = movement.getDirections();
        for (DcMotor motor: motors){
            int x = Arrays.asList(motors).indexOf(motor);
            motor.setPower(signs[x]* speed);

        }
    }
    public void stopDrivetrain() throws InterruptedException{
        for (DcMotor motor: drivetrain){
            motor.setPower(0);
        }
    }


    public void powerMotors(double speed, long time, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(speed);
        }
        central.sleep(time);
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    public void sample() {
        if (sampleStatus==SamplingOrder.UNCHECKED) {
            //Sample
        }
        else if (sampleStatus==SamplingOrder.UNKNOWN){
            //Multiple Attempt Sampling
        }

    }


    //ENUMERATIONS

    public enum EncoderMode{
        ON, OFF
    }
    public enum setupType{
        autonomous, drive, latching, latchingTele, imu, marker, phoneswivel, sensors, mineralControl, teleop, none, vuforia, tensorflow;
    }



    //-------------------SET FUNCTIONS--------------------------------
    public void setCentral(Central central) {
        this.central = central;
    }

    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void setRuntime(ElapsedTime runtime) {
        this.runtime = runtime;
    }



    public void setRack(DcMotor rack) {
        this.rack = rack;
    }

    public void setArm(DcMotor arm) {
        this.arm = arm;
    }
    //-------------------CHOICE ENUMS-------------------------


    public enum movements{
        backward(-1, 1, -1, 1),
        forward(1, -1, 1, -1),
        left(1, 1, -1, -1),
        right(-1, -1, 1, 1),
        tr(0, -1, 1, 0),
        tl(1, 0, 0, -1),
        br(-1, 0, 0, 1),
        bl(0, 1, -1, 0),
        ccw(1, 1, 1, 1),
        cw(-1, -1, -1, -1),
        cwback(-1,-1,0,0),
        ccwback(1,1,0,0),
        cwfront(0,0,-1,-1),
        ccwfront(0,0,1,1),
        rackExtend(1),
        rackCompress(-1),
        forward2(1, -1),
        back2(-1, 1),
        cw2(1,1),
        ccw2(-1, -1);


        private final double[] directions;

        movements(double... signs){
            this.directions = signs;
        }

        public double[] getDirections(){
            return directions;
        }
    }

    // movement but now its better???

    public double[] superturn(double angvelo) { //
        double coeff = angvelo * (1 - StrafetoTotalPower);
        double[] retval = {coeff, -coeff};
        return retval;
    }


    public double[] superstrafe(double dir, double velo, double angvelo){
        double angle = (360+getDirection())%360;
        double[] comp1=anyDirection(velo,dir-angle);
        double[] comp2=superturn(angvelo);
        double[] retval = new double[2];
        for(int i=0;i<4;i++) {
            //drivetrain[i].setPower(comp1[i%2]+comp2[i%2]);
            retval[i%2] = comp1[i%2]+comp2[i%2];
        }
        return retval;
    }


    public static double[] anyDirection(double speed, double angleDegrees){
        double theta = Math.toRadians(angleDegrees);
        double beta = Math.atan(7.0/3.0);

        double v1 = 21.0/58.0 * (speed * Math.sin(theta)/Math.sin(beta) + speed * Math.cos(theta)/Math.cos(beta));
        double v2 = 21.0/58.0 * (speed * Math.sin(theta)/Math.sin(beta) - speed * Math.cos(theta)/Math.cos(beta));

        double[] retval = {v1,v2};
        return retval;
    }
    public static double[] anyDirectionRadians(double speed, double angleRadians){
        double theta = angleRadians;
        double beta = Math.atan(7.0/3.0);

        double v1 = 21.0/58.0 * (speed * Math.sin(theta)/Math.sin(beta) + speed * Math.cos(theta)/Math.cos(beta));
        double v2 = 21.0/58.0 * (speed * Math.sin(theta)/Math.sin(beta) - speed * Math.cos(theta)/Math.cos(beta));

        double[] retval = {v1,v2};
        return retval;
    }
    public void driveTrainMovementAngle(double speed, double angle){

        double[] speeds = anyDirection(speed, angle);
        for (int i = 0; i < drivetrain.length; i++) {
            if (i == 0 || i== 4){
                drivetrain[i].setPower(speeds[1]);
            }
            else {
                drivetrain[i].setPower(speeds[0]);
            }
        }

    }
    public void driveTrainMovementAngleRadians(double speed, double angle){

        double[] speeds = anyDirectionRadians(speed, angle);
        motorFR.setPower(movements.forward.directions[0] * speeds[0]);
        motorFL.setPower(movements.forward.directions[1] * speeds[1]);
        motorBR.setPower(movements.forward.directions[2] * speeds[1]);
        motorBL.setPower(movements.forward.directions[3] * speeds[0]);

    }


    public enum turnside {
        ccw, cw
    }

    public enum axis {
        front, center, back
    }


    //-----------------------------------Image Processing---------------------------------------
    public enum SamplingOrder{
        LEFT, CENTER, RIGHT, UNKNOWN, UNCHECKED;
    }




    //-----------------------------------Mapping------------------------------------------------
    public static class Position{
        double[] vector = {0,0,0};
        double orient;

        public Position(double[] vector, double orient) {
            this.vector = vector;
            this.orient = orient;
        }
        double[] returnv(){
            return vector;

        }
        double returno(){

            return orient;
        }

        void updateOrient(double o){
            orient = o;

        }


    }

    public Position getCurrentPosition() throws InterruptedException{
        double orient =0;
        while(vuforia.checkVisibility().equals("false") && central.opModeIsActive()) {
        }
          //  if(vuforia.checkVisibility().equals("true") && central.opModeIsActive()) {

            VectorF translation = vuforia.lastLocation.getTranslation();

            Orientation rotation = Orientation.getOrientation(vuforia.lastLocation, EXTRINSIC, XYZ, DEGREES);
orient = rotation.thirdAngle;
            return vuftopos((double) (translation.get(0) / mmPerInch), (double) (translation.get(1) / mmPerInch), (double) (translation.get(2) / mmPerInch), rotation.thirdAngle +90 , vuforia.checkVisibility());
      /* }
        else {
            double[] v = {0, 0, 0};
            return new Position(v, 0);
            */
//return currentabspossensors(orient);
       // }
        }
    public static Position vuftopos(double xtrans, double ytrans, double ztrans, double orientation, String id) {
        double v[] = new double[3];

            v[0] = xtrans;
            v[1] = ytrans;
            v[2] = ztrans;

                return new Position(v,orientation);


    }

    public  Position currentabspossensors(double orient){
        //only use when snapped to nearest axis
        double[] w = new double[3];
        if(orient == 90){
          if(rangeDistanceright() > rangeDistanceleft()){
              w[0] = -72 + rangeDistanceleft();
              w[1] = 72-rangeDistancefront();

          }
          else {
              w[0] = 72 - rangeDistanceright();
              w[1] = 72-rangeDistancefront();


          }

        }
        if(orient == 0){
            if(rangeDistanceright() > rangeDistanceleft()){
                w[1] = 72 - rangeDistanceleft();
                w[0] = 72-rangeDistancefront();

            }
            else {
                w[1] = -72 + rangeDistanceright();
                w[0] = 72-rangeDistancefront();


            }

        }
        if(orient == -90){
            if(rangeDistanceright() > rangeDistanceleft()){
                w[0] = 72 - rangeDistanceleft();
                w[1] = -72 + rangeDistancefront();

            }
            else {
                w[0] = -72 + rangeDistanceright();
                w[1] = -72+rangeDistancefront();


            }

        }
        if(orient == 180){
            if(rangeDistanceright() > rangeDistanceleft()){
                w[1] = -72 + rangeDistanceleft();
                w[0] = -72 + rangeDistancefront();

            }
            else {
                w[1] = 72 - rangeDistanceright();
                w[0] = -72+rangeDistancefront();


            }

        }
           return new Position(w,orient);
    }

    public Position motortoabs(Rover.Position p){
        double xval = p.returnv()[0]* Math.cos(p.returno()) - p.returnv()[1]*Math.sin(p.returno());
        double yval = p.returnv()[1]*Math.cos(p.returno()) +  p.returnv()[0]* Math.sin(p.returno());
        double[] a = {xval,yval,p.returnv()[2]};
        return new Rover.Position(a,p.returno());

    }
    public Position abstomotorCoord(Position p){
        double xval = p.returnv()[0]* Math.cos(Math.toRadians(p.returno())) + p.returnv()[1]*Math.sin(Math.toRadians(p.returno()));
        double yval = p.returnv()[1]*Math.cos(Math.toRadians(p.returno())) -  p.returnv()[0]* Math.sin(Math.toRadians(p.returno()));
        double[] a = {xval,yval,p.returnv()[2]};
        return new Position(a,p.returno());

    }
    public Position move(Position startpos, Position endpos) throws InterruptedException{
        double orientMotorcoord = 0;
        Position start = abstomotorCoord(startpos);
        Position end = abstomotorCoord(endpos);
        double dify = end.returnv()[1]- start.returnv()[1];
        double difx = end.returnv()[0]- start.returnv()[0];


        if(difx>=0){
            driveTrainEncoderMovement(0.5,difx,500,100, movements.right);

        }
        else{
            driveTrainEncoderMovement(0.5, Math.abs(difx),500,100, movements.left);


        }
        if(dify>=0){
            driveTrainEncoderMovement(0.5,dify,500,100, movements.forward);

        }
        else{
            driveTrainEncoderMovement(0.5, Math.abs(dify),500,100, movements.backward);


        }
        endpos.updateOrient(endpos.returno() + orientMotorcoord);
        return getCurrentPosition(); //returns abs pos
    }
    public Position moveusingvuf( Position endpos) throws InterruptedException {
     double orientMotorcoord = 0;


        Position end = abstomotorCoord(new Position(endpos.returnv(),getCurrentPosition().returno()));

     if(abstomotorCoord(getCurrentPosition()).returnv()[0] < end.returnv()[0]) {

         central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , abstomotorCoord(getCurrentPosition()).returnv()[0], abstomotorCoord(getCurrentPosition()).returnv()[1],abstomotorCoord(getCurrentPosition()).returno());
         central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , end.returnv()[0], end.returnv()[1],end.returno());
         central.telemetry.update();
         while (Math.abs(abstomotorCoord(getCurrentPosition()).returnv()[0] - end.returnv()[0]) > 5 && central.opModeIsActive()){
             driveTrainMovement(0.05, movements.right);
             central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , abstomotorCoord(getCurrentPosition()).returnv()[0], abstomotorCoord(getCurrentPosition()).returnv()[1],abstomotorCoord(getCurrentPosition()).returno());
             central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , end.returnv()[0], end.returnv()[1],end.returno());
             central.telemetry.update();

         }
     }
     else if(abstomotorCoord(getCurrentPosition()).returnv()[0] > end.returnv()[0]) {
         central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , abstomotorCoord(getCurrentPosition()).returnv()[0], abstomotorCoord(getCurrentPosition()).returnv()[1],abstomotorCoord(getCurrentPosition()).returno());
         central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , end.returnv()[0], end.returnv()[1],end.returno());
         central.telemetry.update();

         while (Math.abs(abstomotorCoord(getCurrentPosition()).returnv()[0] - end.returnv()[0]) > 5 && central.opModeIsActive()){
             driveTrainMovement(0.05, movements.left);
             central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , abstomotorCoord(getCurrentPosition()).returnv()[0], abstomotorCoord(getCurrentPosition()).returnv()[1],abstomotorCoord(getCurrentPosition()).returno());
             central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , end.returnv()[0], end.returnv()[1],end.returno());
             central.telemetry.update();

         }
     }

     if(abstomotorCoord(getCurrentPosition()).returnv()[1] < end.returnv()[1]) {
         central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , abstomotorCoord(getCurrentPosition()).returnv()[0], abstomotorCoord(getCurrentPosition()).returnv()[1],abstomotorCoord(getCurrentPosition()).returno());
         central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , end.returnv()[0], end.returnv()[1],end.returno());
         central.telemetry.update();

         while (Math.abs(abstomotorCoord(getCurrentPosition()).returnv()[1] - end.returnv()[1]) > 5 && central.opModeIsActive()){
             driveTrainMovement(0.05, movements.forward);
             central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , abstomotorCoord(getCurrentPosition()).returnv()[0], abstomotorCoord(getCurrentPosition()).returnv()[1],abstomotorCoord(getCurrentPosition()).returno());
             central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , end.returnv()[0], end.returnv()[1],end.returno());
             central.telemetry.update();

         }
     }
     else if(abstomotorCoord(getCurrentPosition()).returnv()[1] > end.returnv()[1]) {
         central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , abstomotorCoord(getCurrentPosition()).returnv()[0], abstomotorCoord(getCurrentPosition()).returnv()[1],abstomotorCoord(getCurrentPosition()).returno());
         central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , end.returnv()[0], end.returnv()[1],end.returno());
         central.telemetry.update();

         while (Math.abs(abstomotorCoord(getCurrentPosition()).returnv()[1] - end.returnv()[1]) > 5 && central.opModeIsActive()){
             driveTrainMovement(0.05, movements.backward);
             central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , abstomotorCoord(getCurrentPosition()).returnv()[0], abstomotorCoord(getCurrentPosition()).returnv()[1],abstomotorCoord(getCurrentPosition()).returno());
             central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , end.returnv()[0], end.returnv()[1],end.returno());
             central.telemetry.update();

         }
     }
     if(abstomotorCoord(getCurrentPosition()).returno() > endpos.returno()){
         while(Math.abs(abstomotorCoord(getCurrentPosition()).returno() - endpos.returno())>5&& central.opModeIsActive()){
             driveTrainMovement(0.2,cw);
         }

     }
     else if(abstomotorCoord(getCurrentPosition()).returno() < endpos.returno()){
         while(Math.abs(abstomotorCoord(getCurrentPosition()).returno() - endpos.returno())>5&& central.opModeIsActive()){
             driveTrainMovement(0.2,movements.ccw);
         }

     }
     return getCurrentPosition();
 }
    public void ultrasonicturndepot(int degrees, turnside d){

        int a = 45;
        int b = 135-degrees;

        // final/sin45 = start/sinb   final = start(sin45)/sinb
        double start = rangeDistancefront();

        if(d == turnside.ccw) {
            if(rangeDistancefront() < start * Math.sin(45) / Math.sin(b)) {
                while (rangeDistancefront() < start * Math.sin(45) / Math.sin(b)) {
                    motorBR.setPower(0.5);
                    motorFR.setPower(0.5);
                    motorBL.setPower(-0.5);
                    motorFL.setPower(-0.5);

                }
            }
            else if(rangeDistancefront() > start * Math.sin(45) / Math.sin(b)){
                while (rangeDistancefront() > start * Math.sin(45) / Math.sin(b)) {
                    motorBR.setPower(0.5);
                    motorFR.setPower(0.5);
                    motorBL.setPower(-0.5);
                    motorFL.setPower(-0.5);

                }
            }
        }


        else if(d == turnside.cw) {
            if(rangeDistancefront() < start * Math.sin(45) / Math.sin(b)) {
                while (rangeDistancefront() < start * Math.sin(45) / Math.sin(b)) {
                    motorBR.setPower(-0.5);
                    motorFR.setPower(-0.5);
                    motorBL.setPower(0.5);
                    motorFL.setPower(0.5);

                }
            }
            else if(rangeDistancefront() > start * Math.sin(45) / Math.sin(b)){
                while (rangeDistancefront() > start * Math.sin(45) / Math.sin(b)) {
                    motorBR.setPower(-0.5);
                    motorFR.setPower(-0.5);
                    motorBL.setPower(0.5);
                    motorFL.setPower(0.5);

                }
            }

        }

    }

    public Position vufmovetest(Position endpos) throws InterruptedException {


        double orientMotorcoord = 0;
        endpos.updateOrient(getCurrentPosition().returno());

        Position end = abstomotorCoord(new Position(endpos.returnv(),getCurrentPosition().returno()));
central.telemetry.addData("current position","{x, y, orient} = %.0f, %.0f, %.0f" , getCurrentPosition().returnv()[0], getCurrentPosition().returnv()[1],getCurrentPosition().returno());
        central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , abstomotorCoord(getCurrentPosition()).returnv()[0], abstomotorCoord(getCurrentPosition()).returnv()[1],abstomotorCoord(getCurrentPosition()).returno());
        central.telemetry.update();

        if(abstomotorCoord(getCurrentPosition()).returnv()[0]< end.returnv()[0]) {

            while (Math.abs(abstomotorCoord(getCurrentPosition()).returnv()[0] - end.returnv()[0]) > 2 && central.opModeIsActive()){
                central.telemetry.addData("current position","{x, y, orient} = %.0f, %.0f, %.0f" , getCurrentPosition().returnv()[0], getCurrentPosition().returnv()[1],getCurrentPosition().returno());
                central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , abstomotorCoord(getCurrentPosition()).returnv()[0], abstomotorCoord(getCurrentPosition()).returnv()[1],abstomotorCoord(getCurrentPosition()).returno());
                central.telemetry.update();

            }
            central.telemetry.addLine("got it x");
            central.telemetry.update();

        }
        else if(abstomotorCoord(getCurrentPosition()).returnv()[0] > end.returnv()[0]) {

            while (Math.abs(abstomotorCoord(getCurrentPosition()).returnv()[0] - end.returnv()[0]) > 2 && central.opModeIsActive()){
                central.telemetry.addData("current position","{x, y, orient} = %.0f, %.0f, %.0f" , getCurrentPosition().returnv()[0], getCurrentPosition().returnv()[1],getCurrentPosition().returno());
                central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , abstomotorCoord(getCurrentPosition()).returnv()[0], abstomotorCoord(getCurrentPosition()).returnv()[1],abstomotorCoord(getCurrentPosition()).returno());
                central.telemetry.update();

            }
            central.telemetry.addLine("got it x");
            central.telemetry.update();

        }

        if(abstomotorCoord(getCurrentPosition()).returnv()[1] < end.returnv()[1]  ) {

            while (Math.abs(abstomotorCoord(getCurrentPosition()).returnv()[1] - end.returnv()[1]) >2 && central.opModeIsActive()){
                central.telemetry.addData("current position","{x, y, orient} = %.0f, %.0f, %.0f" , getCurrentPosition().returnv()[0], getCurrentPosition().returnv()[1],getCurrentPosition().returno());
                central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , abstomotorCoord(getCurrentPosition()).returnv()[0], abstomotorCoord(getCurrentPosition()).returnv()[1],abstomotorCoord(getCurrentPosition()).returno());
                central.telemetry.update();

            }
            central.telemetry.addLine("got it y");
            central.telemetry.update();

        }
        else if(abstomotorCoord(getCurrentPosition()).returnv()[1] > end.returnv()[1]) {

            while (Math.abs(abstomotorCoord(getCurrentPosition()).returnv()[1] - end.returnv()[1]) > 2 && central.opModeIsActive()){
                central.telemetry.addData("current position","{x, y, orient} = %.0f, %.0f, %.0f" , getCurrentPosition().returnv()[0], getCurrentPosition().returnv()[1],getCurrentPosition().returno());
                central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , abstomotorCoord(getCurrentPosition()).returnv()[0], abstomotorCoord(getCurrentPosition()).returnv()[1],abstomotorCoord(getCurrentPosition()).returno());
                central.telemetry.update();

            }
            central.telemetry.addLine("got it y");
            central.telemetry.update();

        }
        if(abstomotorCoord(getCurrentPosition()).returno() > endpos.returno()){
            while(Math.abs(abstomotorCoord(getCurrentPosition()).returno() - endpos.returno()) > 5 && central.opModeIsActive()){
                central.telemetry.addData("current position","{x, y, orient} = %.0f, %.0f, %.0f" , getCurrentPosition().returnv()[0], getCurrentPosition().returnv()[1],getCurrentPosition().returno());
                central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , abstomotorCoord(getCurrentPosition()).returnv()[0], abstomotorCoord(getCurrentPosition()).returnv()[1],abstomotorCoord(getCurrentPosition()).returno());
                central.telemetry.update();

            }
            central.telemetry.addLine("got it orient");
            central.telemetry.update();


        }
        else if(abstomotorCoord(getCurrentPosition()).returno() < endpos.returno()){
            while(Math.abs(abstomotorCoord(getCurrentPosition()).returno() - endpos.returno()) > 5 && central.opModeIsActive()){
                central.telemetry.addData("current position","{x, y, orient} = %.0f, %.0f, %.0f" , getCurrentPosition().returnv()[0], getCurrentPosition().returnv()[1],getCurrentPosition().returno());
                central.telemetry.addData("current motor position","{x, y, orient} = %.0f, %.0f, %.0f" , abstomotorCoord(getCurrentPosition()).returnv()[0], abstomotorCoord(getCurrentPosition()).returnv()[1],abstomotorCoord(getCurrentPosition()).returno());
                central.telemetry.update();

            }
            central.telemetry.addLine("got it orient");
            central.telemetry.update();


        }
        return getCurrentPosition();
    }
    public Position bettermove( Position endpos) throws InterruptedException {
        double orientMotorcoord = 0;


        Position end = abstomotorCoord(new Position(endpos.returnv(),getCurrentPosition().returno()));
        while (Math.sqrt(Math.pow(Math.abs(abstomotorCoord(getCurrentPosition()).returnv()[0] - end.returnv()[0]),2)+Math.pow(Math.abs(abstomotorCoord(getCurrentPosition()).returnv()[0] - end.returnv()[0]),2)) > 3 && central.opModeIsActive()) {

            if (abstomotorCoord(getCurrentPosition()).returnv()[0] < end.returnv()[0]) {

                driveTrainMovement(0.5, movements.right);


            } else if (abstomotorCoord(getCurrentPosition()).returnv()[0] > end.returnv()[0]) {

                driveTrainMovement(0.5, movements.left);


            } else if (abstomotorCoord(getCurrentPosition()).returnv()[1] < end.returnv()[1]) {

                driveTrainMovement(0.5, movements.forward);


            } else if (abstomotorCoord(getCurrentPosition()).returnv()[1] > end.returnv()[1]) {

                driveTrainMovement(0.5, movements.backward);


            }
        }
        if(abstomotorCoord(getCurrentPosition()).returno() > endpos.returno()){
            while(Math.abs(abstomotorCoord(getCurrentPosition()).returno() - endpos.returno())>5){
                driveTrainMovement(0.5,movements.cw);
            }

        }
        else if(abstomotorCoord(getCurrentPosition()).returno() < endpos.returno()){
            while(Math.abs(abstomotorCoord(getCurrentPosition()).returno() - endpos.returno())>5){
                driveTrainMovement(0.5,movements.ccw);
            }

        }
        return getCurrentPosition();
    }


    //-------------------------------------Sensors-------------------------------------------
    public double getDirection(){
        return (this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle-initorient+720)%360;
    }
}
