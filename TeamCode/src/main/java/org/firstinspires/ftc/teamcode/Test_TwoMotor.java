package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Two Motor Test", group = "Smart")

public class Test_TwoMotor extends TeleOp {

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        setRob(new Rover(hardwareMap, runtime, this, Rover.setupType.drive));
        setRuntime(runtime);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

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
                    rob.motorRight.setPower(fb2);

                } else if (yAxis2 <= -Math.abs(xAxis2)) {
                    rob.motorRight.setPower(-fb2);

                }
            }else{
                rob.motorRight.setPower(0);
            }

        }
    }

}
