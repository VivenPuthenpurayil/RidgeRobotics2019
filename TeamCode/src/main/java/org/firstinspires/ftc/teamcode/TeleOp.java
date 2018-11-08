package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.Constants.DEAD_ZONE_SIZE;

public abstract class TeleOp extends Central{
    public static float yAxis1;
    public static float xAxis1;

    public static float yAxis2;
    public static float xAxis2;

    public static float yRT;
    public static float xRT;

    public static float yLT;
    public static float xLT;

    public static float fb;
    public static float rl;

    public static float fb2;
    public static float rl2;


    public static double diagonalSpeed;

    public static boolean rightStickButtonPressed;
    public static boolean leftStickButtonPressed;

    public void standardGamepadData(){
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

        fb2 = Math.abs(yAxis2);
        rl2 = Math.abs(xAxis2);
    }


    public boolean validStick(double x, double y){
        return Math.pow(x, 2) + Math.pow(y, 2) >= Math.pow(DEAD_ZONE_SIZE, 2);
    }

    public void setMotorPower(double power, DcMotor... motors){
        for (DcMotor b :
                motors) {
            b.setPower(power);
        }
    }


}
