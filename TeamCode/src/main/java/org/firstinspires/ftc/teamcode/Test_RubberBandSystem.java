package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Rubber Band Test", group = "Test")

public class Test_RubberBandSystem extends TeleOp {

    public ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException{
        DcMotor c = hardwareMap.dcMotor.get("motor");


        setRuntime(runtime);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            standardGamepadData();

            if (validStick(xAxis1, yAxis1)) { //MAIN DIRECTIONS

                if (yAxis1 >= Math.abs(xAxis1)) {
                    c.setPower(fb);

                } else if (yAxis1 <= -Math.abs(xAxis1)) {
                    c.setPower(-fb);

                }
            }else{
                c.setPower(0);
            }


        }
    }

}
