package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Rack And Pinion", group = "Smart")

public class Test_RackAndPinion extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    DcMotor rack;


    public void runOpMode() throws InterruptedException{
        rack = motor("rack", DcMotorSimple.Direction.FORWARD);


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            if (gamepad1.a) {
                rack.setPower(0.4);
            }
            else if (gamepad1.y){
                rack.setPower(-0.4);
            }
            else {
                rack.setPower(0);
            }


        }
    }

    public DcMotor motor(String name, DcMotor.Direction direction) throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);

        return motor;
    }
}
