package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    public BNO055IMUImpl imu;

    public HardwareMap hardwareMap;

    DcMotor motorRight;
    DcMotor motorLeft;

    public  DcMotor[] drivetrain = new DcMotor[2];

    public DcMotor motor(String name, DcMotor.Direction direction) throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);
        return motor;
    }

    public void setupDrivetrain() throws InterruptedException {
        motorRight = motor(motorFRS, DcMotorSimple.Direction.FORWARD);
        motorLeft = motor(motorFLS, DcMotorSimple.Direction.FORWARD);

        motorDriveMode(EncoderMode.ON, motorFR, motorFL, motorBR, motorBL);
    }




}
