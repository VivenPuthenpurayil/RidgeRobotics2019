package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Marker Test", group = "Test")

public class Test_Marker extends Test {

    public ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException{
        setRob(new Rover(hardwareMap, runtime, this, Rover.setupType.marker));
        setRuntime(runtime);


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            rob.marker.setPosition(1);
            sleep(1000);
            rob.marker.setPosition(0.7);
            sleep(1000);
            rob.marker.setPosition(0.4);
            sleep(1000);
            rob.marker.setPosition(0.1);
            sleep(1000);
            rob.marker.setPosition(0);
            sleep(1000);
            break;


        }
    }

}
