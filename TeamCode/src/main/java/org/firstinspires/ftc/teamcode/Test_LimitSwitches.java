package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Constants.deployingLimitS;
import static org.firstinspires.ftc.teamcode.Constants.latchingLimitS;


@Autonomous(name = "Limit Switch Test", group = "Test")

public class Test_LimitSwitches extends Test {

    public ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() throws InterruptedException{
        DigitalChannel deployingLimit = hardwareMap.get(DigitalChannel.class, "depLimit");
        DigitalChannel latchingLimit = hardwareMap.digitalChannel.get("latLimit");




        setRuntime(runtime);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            telemetry.addData("Deploying Limit: ", deployingLimit.getState());
            telemetry.addData("Latching Limit: ", latchingLimit.getState());
            telemetry.update();

            sleep(200);


        }
    }

}
