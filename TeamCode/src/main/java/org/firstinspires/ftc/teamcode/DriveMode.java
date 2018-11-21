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
        setRob(new Rover(hardwareMap, runtime, this, Rover.setupType.drive, Rover.setupType.latching, Rover.setupType.mineralControl));
        setRuntime(runtime);


        double initWrist = rob.wrist.getPosition();
        double initElbow = rob.elbow.getPosition();
        double currentWrist = initWrist;
        double currentElbow = initElbow;

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

            if (gamepad1.a && !rob.latchingLimit.getState()) {
                rob.rack.setPower(0.8);
            }
            else if (gamepad1.y && !rob.deployingLimit.getState()){
                rob.rack.setPower(-0.8);
            }
            else {
                rob.rack.setPower(0);
            }

            if (gamepad2.right_trigger > 0.25){
                rob.arm.setPower(0.3);

            }
            else if (gamepad2.left_trigger > 0.25){
                rob.arm.setPower(-0.3);
            }
            else {

                rob.arm.setPower(0);
            }

            /*if (gamepad1.right_trigger>0.25){
                rob.strafer.setPower(0.2);
            }
            else if (gamepad1.left_trigger>0.25){
                rob.strafer.setPower(-0.2);
            }
            else {
                rob.strafer.setPower(0);
            }
*/

            telemetry.addData("Speed:", rob.strafer.getPower());
            telemetry.update();

            if (gamepad2.x){
                currentWrist+=0.01;
                rob.wrist.setPosition(currentWrist);
            }
            else if(gamepad2.b){
                currentWrist-=0.01;
                rob.wrist.setPosition(currentWrist);
            }

            if (gamepad2.y){
                currentElbow+=0.01;
                rob.elbow.setPosition(currentElbow);


            }
            else if (gamepad2.a){
                currentElbow-=0.01;
                rob.elbow.setPosition(currentElbow);
            }






            telemetry.addData("Wrist: ", rob.wrist.getPosition());

            telemetry.addData("Wrist: ", rob.elbow.getPosition());


            telemetry.update();
        }
    }

}
