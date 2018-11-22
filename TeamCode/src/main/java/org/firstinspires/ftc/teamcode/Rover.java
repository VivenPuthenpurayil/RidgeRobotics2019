package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class Rover {
    ElapsedTime runtime;
    Central central;
    public float initorient;
    public CRServo strafer;


    public void setCentral(Central central) {
        this.central = central;
    }

    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void setRuntime(ElapsedTime runtime) {
        this.runtime = runtime;
    }

    public enum movements{
        backward(-1, 1, -1, 1),
        forward(1, -1, 1, -1),
        left(1, 1, -1, -1),
        right(-1, -1, 1, 1),
        tr(0, -1, 1, 0),
        tl(1, 0, 0, -1),
        br(-1, 0, 0, 1),
        bl(0, 1, -1, 0),
        ccw(-1, -1, -1, -1),
        cw(1, 1, 1, 1),
        cwback(-1,-1,0,0),
        ccwback(1,1,0,0),
        cwfront(0,0,-1,-1),
        ccwfront(0,0,1,1),
        rackDown(-1),
        rackUp(1),
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
    public enum turnside {
        ccw, cw
    }

    public enum axis {
        front, center, back
    }
    public HardwareMap hardwareMap;

    public void setRack(DcMotor rack) {
        this.rack = rack;
    }

    public void setArm(DcMotor arm) {
        this.arm = arm;
    }

    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;



    DcMotor rack;
    DcMotor arm;

    Servo marker;
    Servo elbow;
    Servo wrist;

    DigitalChannel latchingLimit;
    DigitalChannel deployingLimit;




    public BNO055IMUImpl imu;
    public BNO055IMUImpl.Parameters parameters = new BNO055IMUImpl.Parameters();
    public Orientation current;

    public static boolean isnotstopped;

    public Rover(setupType... setup) throws InterruptedException {
        for (setupType type : setup) {
            switch (type){
                case drive:
                    setupDrivetrain();
                    break;

                case latching:
                    setupLatching();
                    break;

                case mineralControl:
                    setupMineralControl();
                    break;


                case marker:
                    setupMarker();
                    break;
                case imu:
                    setupIMU();
                    break;

                case vuforia:
                    setupVuforia();
                    break;




            }
        }
    }

    public void initializeLatching() throws InterruptedException{
        while(!latchingLimit.getState())
        {
            anyMovement(0.8, Rover.movements.rackUp, rack);
        }
        rack.setPower(0);
    }

    public void setupVuforia() {

    }


    public  DcMotor[] drivetrain;


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

        strafer = servo(straferS, DcMotorSimple.Direction.FORWARD, 0);

        encoder(EncoderMode.ON, rack);

        initializeLatching();
    }

    public void setupMineralControl() throws InterruptedException{
        arm = motor(armS, DcMotorSimple.Direction.FORWARD);

        elbow = servo(elbowS, Servo.Direction.FORWARD, 0, 1, 0.8);
        wrist = servo(wristS, Servo.Direction.FORWARD, 0, 1, 0.8);

        encoder(EncoderMode.ON, arm);
    }

    public void setupMarker() throws InterruptedException{
        marker = servo(markerS, Servo.Direction.FORWARD, 0, 1, 0.6);
    }

    public void setupIMU() throws InterruptedException{
        parameters.angleUnit = BNO055IMUImpl.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMUImpl.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true; //copypasted from BNO055IMU sample code, no clue what this does
        parameters.loggingTag = "IMU"; //copypasted from BNO055IMU sample code, no clue what this does
        imu = hardwareMap.get(BNO055IMUImpl.class, imuS);
        imu.initialize(parameters);
        initorient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

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
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * distance * COUNTS_PER_INCH);
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
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * distance * COUNTS_PER_INCH);
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

        float start = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        float end = start + ((direction == turnside.cw) ? target : -target);
        isnotstopped = true;
        try {
            switch (rotation_Axis) {
                case center:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cw : movements.ccw);
                    break;
                case back:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwback : movements.ccwback);
                    break;
                case front:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwfront : movements.ccwfront);
                    break;
            }
        } catch (java.lang.InterruptedException e) {
            isnotstopped = false;
        }
        while (!((end <= current.firstAngle + 1) && end > current.firstAngle - 1) && central.opModeIsActive() && isnotstopped) {
            current = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        try {
            stopDrivetrain();
        } catch (java.lang.InterruptedException e) {
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


    //ENUMERATIONS

    public enum EncoderMode{
        ON, OFF
    }
    public enum setupType{
        autonomous, drive, latching, imu, marker, phoneswivel, sensors, mineralControl, teleop, none, vuforia;
    }



}
