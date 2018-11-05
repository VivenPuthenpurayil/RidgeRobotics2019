package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.Constants.*;


@Autonomous(name = "Base Test", group = "Smart")

public class DriveMode extends TeleOp {


    ElapsedTime runtime = new ElapsedTime();
    public void runOpMode() throws InterruptedException{
        setRob(new Rover(hardwareMap, Rover.setupType.drive));
        setRuntime(runtime);

        waitForStart();


        while (opModeIsActive()) {

            // GAMEPAD OBJECT
            standardGamepadData();


            if (validStick(xAxis1, yAxis1)) { //MAIN DIRECTIONS

                if (yAxis1 >= Math.abs(xAxis1)) {
                    rob.motorLeft.setPower(fb);

                } else if (yAxis1 <= -Math.abs(xAxis1)) {
                    rob.motorLeft.setPower(-fb);

                }
            }else{
                rob.motorLeft.setPower(0);
            }

            if (validStick(xAxis2, yAxis2)) { //MAIN DIRECTIONS

                if (yAxis2 >= Math.abs(xAxis2)) {
                    rob.motorRight.setPower(-fb);

                } else if (yAxis2 <= -Math.abs(xAxis2)) {
                    rob.motorRight.setPower(fb);

                    }
            }else{
                rob.motorRight.setPower(0);
            }



        }
    }

}
