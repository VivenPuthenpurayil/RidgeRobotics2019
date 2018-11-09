package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.AutonomousControl;


@Autonomous(name = "Tennis Ball Shoot", group = "Smart")

public class TennisTest extends AutonomousControl {

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        setRob(new Rover(hardwareMap, Rover.setupType.drive));
        setRuntime(runtime);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            //rob.motorLeft.setPower(1);
            //rob.motorRight.setPower(-1);
            sleep(10000);

        }
    }

}
