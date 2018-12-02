package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.TeleOpControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

import static org.firstinspires.ftc.teamcode.Control.Constants.*;


@TeleOp(name = "DriveMode", group = "Smart")

public class DriveMode extends TeleOpControl {


    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode() throws InterruptedException{

        setup(runtime, Rover.setupType.drive, Rover.setupType.latching, Rover.setupType.mineralControl);


        while (opModeIsActive()) {

            // GAMEPAD OBJECT
            standardGamepadData();


            if (leftStickButtonPressed) {
                // CLOCKWISE
                rob.driveTrainMovement(ROTATION_SPEED, Rover.movements.cw);
            } else if (rightStickButtonPressed) {
                // COUNTERCLOCKWISE
                rob.driveTrainMovement(ROTATION_SPEED, Rover.movements.ccw);
            } else if (validStick(xAxis1, yAxis1)) { //MAIN DIRECTIONS

                if (yAxis1 >= Math.abs(xAxis1)) {
                    rob.driveTrainMovement(fb,Rover.movements.forward);
                    //FORWARD
                } else if (yAxis1 <= -Math.abs(xAxis1)) {
                    rob.driveTrainMovement(fb,Rover.movements.backward);
                    //BACKWARD
                } else if (Math.abs(yAxis1) < xAxis1) {
                    rob.driveTrainMovement(rl,Rover.movements.right);
                    //RIGHT
                } else if (-Math.abs(yAxis1) > xAxis1) {
                    rob.driveTrainMovement(rl,Rover.movements.left);
                    //LEFT
                }
            } else if (validStick(xAxis2, yAxis2)) {    //DIAGONAL

                if (yAxis2 >= 0 && xAxis2 >= 0) {
                    rob.driveTrainMovement(diagonalSpeed,Rover.movements.tr);
                    //TOP RIGHT
                } else if (yAxis2 >= 0 && xAxis2 < 0) {
                    rob.driveTrainMovement(diagonalSpeed,Rover.movements.tl);
                    //TOP LEFT
                } else if (yAxis2 < 0 && xAxis2 >= 0) {
                    rob.driveTrainMovement(diagonalSpeed, Rover.movements.br);
                    //BOTTOM RIGHT
                } else if (yAxis2 < 0 && xAxis2 < 0) {
                    rob.driveTrainMovement(diagonalSpeed,Rover.movements.bl);
                    //BOTTOM LEFT
                }
            } else {
                rob.stopDrivetrain();
            }


            if (gamepad1.right_trigger > 0.25){
                rob.linear.setPower(0.4);
            }
            else if (gamepad1.left_trigger > 0.25){
                rob.linear.setPower(-0.4);
            }
            else {
                rob.linear.setPower(0);
            }

            if (gamepad1.x){
                rob.collector.setPower(0.6);
            }
            else if (gamepad1.b){
                rob.collector.setPower(-0.6);
            }
            else {
                rob.collector.setPower(0);
            }


            if (gamepad1.dpad_right){
                rob.arm.setPower(0.4);
            }
            else if (gamepad1.dpad_left){
                rob.arm.setPower(-0.4);
            }
            else {
                rob.arm.setPower(0);
            }


            if(gamepad1.a){
                rob.rack.setPower(1);
            }
            else if (gamepad1.y){
                rob.rack.setPower(-1);
            }
            else {
                rob.rack.setPower(0);
            }
        }
    }

}
