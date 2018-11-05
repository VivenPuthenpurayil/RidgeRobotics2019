package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name = "Base Test", group = "Smart")

public class BaseTest extends LinearOpMode {

    private static final double DEAD_ZONE_SIZE = 0.2;
    public ElapsedTime runtime = new ElapsedTime();
    DcMotor left, right;

    public static float yAxis1;
    public static float xAxis1;

    public static float yAxis2;
    public static float xAxis2;

    public static float fb;
    public static float rl;



    public void runOpMode() throws InterruptedException{
        left = motor("left", DcMotorSimple.Direction.FORWARD);
        right = motor("right", DcMotorSimple.Direction.REVERSE);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            yAxis1 = -gamepad1.left_stick_y; // Main Directions y-axis
            xAxis1 = gamepad1.left_stick_x;  // Main Directions x-axis

            yAxis2 = -gamepad1.right_stick_y; // Diagonal Directions y-axis
            xAxis2 = gamepad1.right_stick_x;  // Diagonal Directions x-axis

            yAxis1 = Range.clip(yAxis1, -1, 1);
            xAxis1 = Range.clip(xAxis1, -1, 1);

            yAxis2 = Range.clip(yAxis2, -1, 1);
            xAxis2 = Range.clip(xAxis2, -1, 1);


            fb = Math.abs(yAxis1);
            rl = Math.abs(xAxis1);


            if (Math.pow(xAxis1, 2) + Math.pow(yAxis1, 2) >= Math.pow(DEAD_ZONE_SIZE, 2)) { //MAIN DIRECTIONS

                if (yAxis1 >= Math.abs(xAxis1)) {
                    left.setPower(fb);

                } else if (yAxis1 <= -Math.abs(xAxis1)) {
                    left.setPower(-fb);

                }
            }else{
                left.setPower(0);
            }

            if (Math.pow(xAxis2, 2) + Math.pow(yAxis2, 2) >= Math.pow(DEAD_ZONE_SIZE, 2)) { //MAIN DIRECTIONS

                if (yAxis2 >= Math.abs(xAxis2)) {
                    right.setPower(-fb);

                } else if (yAxis2 <= -Math.abs(xAxis2)) {
                    right.setPower(fb);


            }
            }else{
                right.setPower(0);
            }



        }
    }

    public DcMotor motor(String name, DcMotor.Direction direction) throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);

        return motor;
    }
}
