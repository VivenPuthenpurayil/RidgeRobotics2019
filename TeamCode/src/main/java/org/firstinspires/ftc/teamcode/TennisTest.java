package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Tennis Ball Shoot", group = "Smart")

public class TennisTest extends AutonomousControl {

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        setRob(new Rover(hardwareMap, Rover.setupType.drive));
        setRuntime(runtime);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            rob.motorLeft.setPower(1);
            rob.motorRight.setPower(-1);
            sleep(10000);

        }
    }

}
