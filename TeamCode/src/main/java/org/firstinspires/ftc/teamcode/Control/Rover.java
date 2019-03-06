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

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_MOTOR_REV;
import static org.firstinspires.ftc.teamcode.Control.Constants.armS;
import static org.firstinspires.ftc.teamcode.Control.Constants.collectorS;
import static org.firstinspires.ftc.teamcode.Control.Constants.deployingLimitS;
import static org.firstinspires.ftc.teamcode.Control.Constants.imuS;
import static org.firstinspires.ftc.teamcode.Control.Constants.latchingLimitS;
import static org.firstinspires.ftc.teamcode.Control.Constants.linearS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorBLS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorBRS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorFLS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorFRS;
import static org.firstinspires.ftc.teamcode.Control.Constants.phoneSwivelS;
import static org.firstinspires.ftc.teamcode.Control.Constants.rackS;
import static org.firstinspires.ftc.teamcode.Control.Rover.movements.backward;
import static org.firstinspires.ftc.teamcode.Control.Rover.movements.ccw;
import static org.firstinspires.ftc.teamcode.Control.Rover.movements.cw;
import static org.firstinspires.ftc.teamcode.Control.Rover.movements.forward;
import static org.firstinspires.ftc.teamcode.Control.Rover.movements.left;
import static org.firstinspires.ftc.teamcode.Control.Rover.movements.right;

public class Rover {

    public Rover(HardwareMap hardwareMap, ElapsedTime runtime, Central central, setupType... setup) throws InterruptedException {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.central = central;

        StringBuilder i = new StringBuilder();
        for (setupType type : setup) {
            switch (type) {
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
                    setupVuforia(VuforiaHandler.type.images);
                    break;

                case tensorflow:
                    setupVuforia(VuforiaHandler.type.minerals);
                    break;

                case fullvision:
                    setupVuforia(VuforiaHandler.type.both);
                    break;

                case positioning:
                    setupPositionProcessing(vuforiaMode);
                    break;
                case sensors:
                    setupSensors();
                    break;


                case autonomous:
                    setupLatching();
                    setupIMU();
                    setupDrivetrain();
                    setupMineralControl();
                    setupVuforia(VuforiaHandler.type.both);
                    setupPhone();
                    //setupSensors();
                    break;


                case phoneswivel:
                    setupPhone();
                    break;
            }
            i.append(type.name()).append(" ");
        }
        central.telemetry.addLine(i.toString());
        central.telemetry.update();


    }

    public void setupLatchingTele() throws InterruptedException {
        rack = motor(rackS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);

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

    static final double MAX_POS = 0.6;     // Maximum rotational position
    static final double MIN_POS = 0.2;     // Minimum rotational position

    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;

    double position = 0;

    boolean targetVisible;


    public int[] wheelAdjust = {1, 1, 1, 1};

    public static double speedAdjust = 20.0 / 41.0;
    public static double yToXRatio = 1.25;

    public void setWheelAdjust(int fr, int fl, int br, int bl) {
        wheelAdjust[0] = fr;
        wheelAdjust[1] = fl;
        wheelAdjust[2] = br;
        wheelAdjust[3] = bl;
    }
    //----specfic non-configuration fields
    //none rn



    //----------------CONFIGURATION FIELDS--------------------


    SamplingOrder sampleStatus = SamplingOrder.UNCHECKED;



    //---- ARM ----------
    //AnalogInput armPotentiometer = hardwareMap.analogInput.get("potentiometer");
    private double maxPotentiometerVal = 5;
    private double minPotentiometerVal = 0;
    //----  MAPPING         ----
    ModernRoboticsI2cRangeSensor FRU;
    ModernRoboticsI2cRangeSensor FLU;
    ModernRoboticsI2cRangeSensor BCU;
    ModernRoboticsI2cRangeSensor BRU;
    ModernRoboticsI2cRangeSensor BLU;


    //----  DRIVE           ----
    public DcMotor[] drivetrain;   //set in motorDriveMode() for drivetrain movement functions

    public DcMotor motorFR;
    public DcMotor motorFL;
    public DcMotor motorBR;
    public DcMotor motorBL;
    public DcMotor leftshooter;
    public DcMotor rightshooter;
    public DcMotor collector1;



    public double StrafetoTotalPower = 2.0/3.0;
    public double mecanumAngle = 36; //from forwards, in degrees
    public double communism = StrafetoTotalPower * Math.cos(Math.toRadians(mecanumAngle * 2));

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
    public Servo servo;

    //---- VUFORIA HANDLER  ----
    public VuforiaHandler vuforia;
    public boolean vuforiaMode;
    public boolean tensorflowMode;
    public float initorient;

    //---- VUFORIA POSITIONING

    PositionProcessor processor;


    //-----         LATCHING FUNCTIONS          --------------
    public void latchInit() throws InterruptedException {
        while (!latchingLimit.getState()) {
            anyMovement(0.8, Rover.movements.rackCompress, rack);
        }
        rack.setPower(0);
        central.telemetry.addLine("Latched");
        central.telemetry.update();
    }

    public void latch() throws InterruptedException {
        while (!latchingLimit.getState() && central.opModeIsActive()) {
            anyMovement(0.8, Rover.movements.rackCompress, rack);
        }
        rack.setPower(0);
        central.telemetry.addLine("Latched");
        central.telemetry.update();
    }

    public void deploy() throws InterruptedException {

        while (!deployingLimit.getState() && central.opModeIsActive()) {
            anyMovement(0.8, movements.rackExtend, rack);
        }
        rack.setPower(0);
        //driveTrainEncoderMovement(0.8, 0.5, 3, 50, cw);
        //while(Math.absimu.getAcceleration())
        driveTrainEncoderMovement(0.3, 0.5, 5, 0, backward);
        driveTrainEncoderMovement(0.5, 1, 5, 0, left);
        driveTrainEncoderMovement(0.3, 0.75, 5, 0, forward);

        //driveTrainEncoderMovement(0.8, 5, 3, 50, ccw);
        //driveTrainEncoderMovement(0.8, 2, 3, 50, backward);

    }

    //------        MAPPING FUNCTIONS           --------------

    public double rangeDistanceFRU() {
        return FRU.getDistance(DistanceUnit.INCH);
    }

    public double rangeDistanceFLU() {
        return FLU.getDistance(DistanceUnit.INCH);
    }

    public double rangeDistanceBRU() {
        return BRU.getDistance(DistanceUnit.INCH);
    }

    public double rangeDistanceBLU() {
        return BLU.getDistance(DistanceUnit.INCH);
    }

    public double rangeDistanceBCU() {
        return BCU.getDistance(DistanceUnit.INCH);
    }


    //----          SETUP FUNCTIONS             --------------

    public void setupPhone() throws InterruptedException {
        servo = servo(phoneSwivelS, Servo.Direction.FORWARD, 0, 1, 0.47);

    }

    public void setupSensors() {


        FRU = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "frontRange");
        FLU = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "backRange");

        BCU = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRange");
        BRU = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftRange");
    }

    public void setupVuforia(VuforiaHandler.type s) {
        if (s == VuforiaHandler.type.both) {
            vuforia = new VuforiaHandler(central, s);
            vuforiaMode = true;
            tensorflowMode = true;
        }
        else if (s == VuforiaHandler.type.images){
            vuforia = new VuforiaHandler(central, s);
            vuforiaMode = true;
        }
        else {
            vuforia = new VuforiaHandler(central, s);
            tensorflowMode = true;
        }

    }
    public void setupPositionProcessing(boolean vuforiaMode){
        if (vuforiaMode) {
            processor = new PositionProcessor(vuforia);
        }
        else {
            processor = new PositionProcessor();
        }

    }

    public void setupDrivetrain() throws InterruptedException {
        motorFR = motor(motorFRS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        motorFL = motor(motorFLS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        motorBR = motor(motorBRS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        motorBL = motor(motorBLS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);

        motorDriveMode(EncoderMode.ON, motorFR, motorFL, motorBR, motorBL);
    }

    public void setupLatching() throws InterruptedException {
        rack = motor(rackS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);

        deployingLimit = hardwareMap.digitalChannel.get(deployingLimitS);//name it limit in config pls <3
        latchingLimit = hardwareMap.digitalChannel.get(latchingLimitS);

        encoder(EncoderMode.ON, rack);

        latchInit();
    }

    public void setupMineralControl() throws InterruptedException{
        arm = motor(armS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);

        linear = motor(linearS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
        collector = motor(collectorS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);


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
    public DcMotor motor(String name, DcMotor.Direction directionm, DcMotor.ZeroPowerBehavior zeroPowerBehavior) throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
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
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * wheelAdjust[x] * distance * COUNTS_PER_MOTOR_REV);
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
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * wheelAdjust[x] * distance * 4.0 * 3.14165 * COUNTS_PER_INCH);
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
    public int checkMinerals(int gold, int sil1, int sil2){
        if (gold == -1 && sil1 != -1 && sil2 != -1){
            return 0;
        }
        else if (gold != -1 && sil1 == -1 && sil2 != -1){
            return 1;
        }
        else if (gold != -1 && sil1 != -1 && sil2 == -1){
            return 2;
        }else{
            return -1;
        }

    }
    public void turn(float target, turnside direction, double speed, axis rotation_Axis) throws InterruptedException{

        central.telemetry.addData("IMU State: ", imu.getSystemStatus());
        central.telemetry.update();

        double start = getDirection();

        double end = (start + ((direction == turnside.cw) ? target : -target) + 360) % 360;

        isnotstopped = true;
        try {
            switch (rotation_Axis) {
                case center:
                    driveTrainMovement(speed, (direction != turnside.cw) ? cw : ccw);
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

        while (!((end <= getDirection()+1) && end > getDirection() - 1) && central.opModeIsActive() && isnotstopped) {
            if (end + 1 < getDirection()){
                driveTrainMovement(0.1, (direction == turnside.cw) ? ccw : movements.cw);
            }
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());
            central.telemetry.update();
        }
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
            motor.setPower(signs[x] * wheelAdjust[x]* speed);

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
            motor.setPower(signs[x] * wheelAdjust[x]* speed);

        }
        central.sleep(duration);
        stopDrivetrain();
        central.sleep(waitAfter);
    }

    public void anyMovement(double speed, movements movement, DcMotor... motors) throws InterruptedException{
        double[] signs = movement.getDirections();
        for (DcMotor motor: motors){
            int x = Arrays.asList(motors).indexOf(motor);
            motor.setPower(signs[x] * wheelAdjust[x]* speed);

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
    public void shoot(){




    }
    public void sample() {
        if (sampleStatus==SamplingOrder.UNCHECKED) {
            //Sample
        }
        else if (sampleStatus==SamplingOrder.UNKNOWN){
            //Multiple Attempt Sampling
        }

    }

    public void armMoveUp(){
        // arm.setPower(1-Math.abs((armPotentiometer.getVoltage()-minPotentiometerVal)/(maxPotentiometerVal-minPotentiometerVal)-1));
    }

    public void armMoveDown() {
        //arm.setPower(Math.abs((armPotentiometer.getVoltage()-minPotentiometerVal)/(maxPotentiometerVal-minPotentiometerVal)-1)-1);
    }
    //ENUMERATIONS

    public enum EncoderMode {
        ON, OFF
    }
    public enum setupType{
        autonomous, drive, latching, latchingTele, imu, marker, phoneswivel, sensors, mineralControl, teleop, none, vuforia, fullvision, tensorflow,  positioning;
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


    public enum movements {
        backward(-1, 1, -1, 1),
        forward(1, -1, 1, -1),
        left(1, 1, -1, -1),
        right(-1, -1, 1, 1),
        tr(0, -1, 1, 0),
        tl(1, 0, 0, -1),
        bl(0, 1, -1, 0),
        br(-1, 0, 0, 1),
        ccw(1, 1, 1, 1),
        cw(-1, -1, -1, -1),
        cwback(-1, -1, 0, 0),
        ccwback(1, 1, 0, 0),
        cwfront(0, 0, -1, -1),
        ccwfront(0, 0, 1, 1),
        rackExtend(-1),
        rackCompress(1),
        forward2(1, -1),
        back2(-1, 1),
        cw2(1, 1),
        ccw2(-1, -1),
        linearOut(-1),
        linearIn(1),
        armUp(1),
        armDown(-1),
        collectorEject(1);



        private final double[] directions;

        movements(double... signs) {
            this.directions = signs;
        }

        public double[] getDirections() {
            return directions;
        }
    }

    // movement but now its better???

    public double[] superturn(double angvelo, movements tdir) { //
        double coeff = angvelo * (1 - StrafetoTotalPower);
        double[] retval = {tdir.getDirections()[0]*coeff, tdir.getDirections()[0]*coeff};
        return retval;
    }


    public double[] superstrafe(double dir, double velo, double angvelo, movements tdir) {
        double[] comp1 = anyDirection(velo, dir - getDirection());
        double[] comp2 = superturn(angvelo,tdir);
        double[] retval = new double[2];
        for (int i = 0; i < 4; i++) {
            drivetrain[i].setPower(comp1[i%2]+comp2[i%2]);
            retval[i % 2] = comp1[i % 2] + comp2[i % 2];
        }
        return retval;
    }


    public static double[] anyDirection(double speed, double angleDegrees) {
        double theta = Math.toRadians(angleDegrees);
        double beta = Math.atan(yToXRatio);

        double v1 = speedAdjust * (speed * Math.sin(theta) / Math.sin(beta) + speed * Math.cos(theta) / Math.cos(beta));
        double v2 = speedAdjust * (speed * Math.sin(theta) / Math.sin(beta) - speed * Math.cos(theta) / Math.cos(beta));

        double[] retval = {v1, v2};
        return retval;
    }

    public static double[] anyDirectionRadians(double speed, double angleRadians) {
        double theta = angleRadians;
        double beta = Math.atan(yToXRatio);

        double v1 = speedAdjust * (speed * Math.sin(theta) / Math.sin(beta) + speed * Math.cos(theta) / Math.cos(beta));
        double v2 = speedAdjust * (speed * Math.sin(theta) / Math.sin(beta) - speed * Math.cos(theta) / Math.cos(beta));

        double[] retval = {v1, v2};
        return retval;
    }

    public void driveTrainMovementAngle(double speed, double angle) {

        double[] speeds = anyDirection(speed, angle);
        for (int i = 0; i < drivetrain.length; i++) {
            if (i == 0 || i == 4) {
                drivetrain[i].setPower(speeds[1]);
            } else {
                drivetrain[i].setPower(speeds[0]);
            }
        }

    }

    public void driveTrainMovementAngleRadians(double speed, double angle) {

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

    public PositionProcessor.Point getCurrentPosition() throws InterruptedException{
        String r = vuforia.checkVisibility();
        if(r.equals("false")){
            return new PositionProcessor.Point(new VectorF(-1000,-1000,-1000), new Orientation(), "false");
        }

        VectorF translation = vuforia.lastLocation.getTranslation();

        Orientation rotation = Orientation.getOrientation(vuforia.lastLocation, EXTRINSIC, XYZ, DEGREES);
        return processor.vuforiaInput(translation, rotation, r);
    }


    public void phoneCheck() throws InterruptedException {
        String value = vuforia.checkVisibility();

        switch (value){
            case "false":
                central.telemetry.addLine("No Visible Image");
                central.telemetry.update();
                break;
            default:
                VectorF translation = vuforia.lastLocation.getTranslation();
                central.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(vuforia.lastLocation, EXTRINSIC, XYZ, DEGREES);
                central.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                processor.vuforiaInput(translation, rotation, value);
                double angle = processor.phoneMountAngle();
                servo.setPosition(angle);
                central.telemetry.addData("Phone mount pos: ", angle);

                central.telemetry.addData("Angle: ", Math.toDegrees((angle - 0.47)* Math.toRadians(300)));
                central.telemetry.update();
                central.idle();
                break;
        }
    }

    public void ultrasonicParallelToWall(double speed ,double tolerance) throws InterruptedException {
        double FRUDistance = FRU.getDistance(DistanceUnit.INCH);
        double FLUDistance = FLU.getDistance(DistanceUnit.INCH);

        if(FRUDistance > FLUDistance) {
            while(FLUDistance+tolerance>FRUDistance){
                driveTrainMovement(speed, ccw);
                FRUDistance = FRU.getDistance(DistanceUnit.INCH);
                FLUDistance = FLU.getDistance(DistanceUnit.INCH);
            }
        }else if(FLUDistance > FRUDistance){
            while(FRUDistance+tolerance>FLUDistance){
                driveTrainMovement(speed, cw);
                FRUDistance = FRU.getDistance(DistanceUnit.INCH);
                FLUDistance = FLU.getDistance(DistanceUnit.INCH);
            }
        }else {
            central.telemetry.addLine("No bueno");
        }
    }

    public void ultrasonicMoveRightParallelToWall(double speed, double tolerance, double stopDistanceAway, movements movements) throws InterruptedException {
        double FRUDistance = FRU.getDistance(DistanceUnit.INCH);
        double FLUDistance = FLU.getDistance(DistanceUnit.INCH);
        double BRUDistance = BRU.getDistance(DistanceUnit.INCH);

        if(BRUDistance<stopDistanceAway){
            while((FRUDistance-FLUDistance)<tolerance && BRUDistance<stopDistanceAway) {
                FRUDistance = FRU.getDistance(DistanceUnit.INCH);
                FLUDistance = FLU.getDistance(DistanceUnit.INCH);
                BRUDistance = BRU.getDistance(DistanceUnit.INCH);

                driveTrainMovement(speed, movements);
            }
        }
    }


    //-------------------------------------Sensors-------------------------------------------
    public double getDirection(){
        return (this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle-initorient+720)%360;
    }


}
