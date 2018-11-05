package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Tennis Ball Shoot", group = "Smart")

public class TennisTest extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        left = motor("left", DcMotorSimple.Direction.FORWARD);
        right = motor("right", DcMotorSimple.Direction.FORWARD);


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            left.setPower(1);
            right.setPower(-1);
            sleep(10000);

        }
    }

    public DcMotor motor(String name, DcMotor.Direction direction) throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);

        return motor;
    }
}
