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

        }
    }

}
