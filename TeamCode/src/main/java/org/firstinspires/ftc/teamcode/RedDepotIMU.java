package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.COUNTS_PER_INCH;

@Autonomous(name = "Red Depot IMU ", group = "A")
public class RedDepotIMU extends AutonomousControl{


    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        setRob(new Rover(hardwareMap, runtime, this, Rover.setupType.drive, Rover.setupType.latching, Rover.setupType.mineralControl, Rover.setupType.marker, Rover.setupType.phoneswivel, Rover.setupType.imu));
        setRuntime(runtime);

        while(!rob.latchingLimit.getState())
        {
            rob.anyMovement(0.8, Rover.movements.rackUp, rob.rack);
        }
        rob.rack.setPower(0);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()){

            //oneSmallStep2();
            //sample();
            /*encoderMovement(0.4, 5, 10, 500, ccw2, rob.motorLeft, rob.motorRight);
            encoderMovement(0.4, 20, 10, 500, forward2, rob.motorLeft, rob.motorRight);
            encoderMovement(0.4, 10, 10, 500, cw2, rob.motorLeft, rob.motorRight);
            encoderMovement(0.4, 20, 10, 500, forward2, rob.motorLeft, rob.motorRight);
            rob.marker.setPosition(1);
            sleep(500);
            rob.marker.setPosition(0.3);
            encoderMovement(0.4, 50, 10, 500, back2, rob.motorLeft, rob.motorRight);

            break;*/
        }
    }
    public void encoderMovement(double speed, double distance, double timeoutS, long waitAfter, Rover.movements movement, DcMotor... motors) throws  InterruptedException{

        int[] targets = new int[motors.length];
        double[] signs = movement.getDirections();
        sleep(4000);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            sleep(4000);
            for (DcMotor motor : motors){
                int x = Arrays.asList(motors).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * distance * COUNTS_PER_INCH);
            }
            for (DcMotor motor: motors){
                int x = Arrays.asList(motors).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:motors){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
                for (DcMotor motor: motors){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            // Stop all motion;
            for (DcMotor motor: motors){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            sleep(waitAfter);


        }
    }

}
