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

        setup(runtime, Rover.setupType.drive);

        waitForStart();


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


        }
    }

}
