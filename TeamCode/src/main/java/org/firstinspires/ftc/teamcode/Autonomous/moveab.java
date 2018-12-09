package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous(name="moveab", group ="Smart")
public class moveab extends AutonomousControl{



    private ElapsedTime runtime = new ElapsedTime();
        @Override
        public void runOpMode() throws InterruptedException {
            setup(runtime, Rover.setupType.autonomous);

            while (opModeIsActive()) {

                double[] x = new double[3];
                x[0] = -40;
                x[1] = -10;
                x[2] = 12;
                double[] y = new double[3];
                x[0] = 5;
                x[1] = -60;
                x[2] = 12;

                rob.move(new Rover.Position(y,90),new Rover.Position(x,90));


            }
        }

    }


