package org.firstinspires.ftc.teamcode;

public class Constants {


    //--------------------------------ENCODERS-------------------------


    public static final double COUNTS_PER_MOTOR_NEVEREST = 1680;
    public static final double COUNTS_PER_MOTOR_TETRIX = 1440;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_NEVEREST * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);                   // Conversion: Encoder Count to Inches
    public static final double COUNTS_PER_TETRIX_INCH = (COUNTS_PER_MOTOR_TETRIX * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);               // Conversion: Encoder Counts Motor Tetrix to Inches


    //--------------------------------TELE-OP VALUES--------------------
    public static final double ROTATION_SPEED = 0.8;
    public static final double DEAD_ZONE_SIZE = 0.1;
    public static final double D_PAD_SPEED = 0.4;
    public static final double CRAWL_SPEED = 0.2;
    public static final double WRIST_SPEED = .01;
    public static final double ELBOW_SPEED = .01;



    //--------------------------------CONFIGURATION VALUES--------------------

    public static final String motorRightS = "motorRight";
    public static final String motorLeftS = "motorLeft";
    public static final String rackS = "rack";
    public static final String armS = "arm";

    public static final String deployingLimitS = "depLimit";
    public static final String latchingLimitS = "latLimit";


    public static final String markerS = "marker";
    public static final String elbowS = "elbow";
    public static final String wristS = "wrist";

    public static final String imuS = "imu";

    public static final String straferS = "strafer";





}
