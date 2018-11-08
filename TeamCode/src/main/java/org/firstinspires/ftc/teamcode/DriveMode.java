package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.Constants.*;


@Autonomous(name = "DriveMode", group = "Smart")

public class DriveMode extends TeleOp {


    ElapsedTime runtime = new ElapsedTime();
    public void runOpMode() throws InterruptedException{
        setRob(new Rover(hardwareMap, Rover.setupType.drive, Rover.setupType.latching, Rover.setupType.mineralControl));
        setRuntime(runtime);

        waitForStart();


        while (opModeIsActive()) {

            // GAMEPAD OBJECT
            standardGamepadData();


            if (validStick(xAxis1, yAxis1)) { //MAIN DIRECTIONS

                if (yAxis1 >= Math.abs(xAxis1)) {
                    rob.motorLeft.setPower(fb);
                    setMotorPower(fb);

                } else if (yAxis1 <= -Math.abs(xAxis1)) {
                    rob.motorLeft.setPower(-fb);

                }
            }else{
                rob.motorLeft.setPower(0);
            }

            if (validStick(xAxis2, yAxis2)) { //MAIN DIRECTIONS

                if (yAxis2 >= Math.abs(xAxis2)) {
                    rob.motorRight.setPower(-fb2);

                } else if (yAxis2 <= -Math.abs(xAxis2)) {
                    rob.motorRight.setPower(fb2);

                    }
            }else{
                rob.motorRight.setPower(0);
            }

            if (gamepad1.a) {
                rob.rack.setPower(0.8);
            }
            else if (gamepad1.y){
                rob.rack.setPower(-0.8);
            }
            else {
                rob.rack.setPower(0);
            }

            if (gamepad1.x){
                rob.arm.setPower(0.9);

            }
            else if (gamepad1.b){
                rob.arm.setPower(-0.9);
            }
            else {

                rob.arm.setPower(0);
            }





        }
    }

}
