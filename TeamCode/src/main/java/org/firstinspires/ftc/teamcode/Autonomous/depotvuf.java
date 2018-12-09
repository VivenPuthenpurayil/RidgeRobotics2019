package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

@Autonomous(name="Depotvuf", group ="Smart")


public class depotvuf extends  AutonomousControl{



        private ElapsedTime runtime = new ElapsedTime();
        @Override
        public void runOpMode() throws InterruptedException {
            setup(runtime, Rover.setupType.autonomous);

            while (opModeIsActive()) {

                double[] x = new double[3];
                x[0] = 5;
                x[1] = 12;
                x[3] = 12;

                rob.moveusingvuf(new Rover.Position(x,90));


            }
        }

}
