package org.firstinspires.ftc.teamcode.Autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Rover;

@Autonomous(name="EncoderCrater", group ="Smart")

public class EncoderCrater extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();
    private double rotationperinch=0.7;
    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Rover.setupType.autonomous);

        while (opModeIsActive()){
            rob.deploy();
            SamplingOrderDetector detector = new SamplingOrderDetector();
            detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
            detector.useDefaults();
            detector.downscale = 0.4;
            detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
            detector.maxAreaScorer.weight = 0.005;
            detector.ratioScorer.weight = 5;
            detector.ratioScorer.perfectRatio = 1.0;
            detector.enable();
            switch(detector.getCurrentOrder()){
                case LEFT:
                case UNKNOWN:
                    rob.turn(35,Rover.turnside.ccw,1,Rover.axis.center);
                    rob.driveTrainEncoderMovement(0.75,35.4*rotationperinch,5,200,Rover.movements.forward);

                    rob.turn(100,Rover.turnside.ccw,1,Rover.axis.center);
                    rob.driveTrainEncoderMovement(0.75,88.4*rotationperinch,5,200,Rover.movements.forward);

                    rob.arm.setPower(0.3);
                    sleep(500);
                    rob.arm.setPower(0);

                    rob.turn(10,Rover.turnside.ccw,1,Rover.axis.center);
                    rob.driveTrainEncoderMovement(0.75,123.8*rotationperinch,5,200,Rover.movements.backward);

                    rob.turn(180,Rover.turnside.ccw,1,Rover.axis.center);
                    rob.linear.setPower(1);
                    sleep(500);
                    rob.linear.setPower(0);
                case CENTER:
                    rob.driveTrainEncoderMovement(0.5,35.3*rotationperinch,5,200,Rover.movements.forward);

                    rob.driveTrainEncoderMovement(0.5,35.4*rotationperinch,5,200,Rover.movements.backward);

                    rob.turn(45,Rover.turnside.ccw,1,Rover.axis.center);
                    rob.driveTrainEncoderMovement(0.75,44.2*rotationperinch,5,200,Rover.movements.forward);

                    rob.turn(90,Rover.turnside.ccw,1,Rover.axis.center);
                    rob.driveTrainEncoderMovement(0.75,70.8*rotationperinch,5,200,Rover.movements.forward);

                    rob.arm.setPower(0.3);
                    sleep(500);
                    rob.arm.setPower(0);

                    rob.driveTrainEncoderMovement(0.75,123.8*rotationperinch,5,200,Rover.movements.backward);

                    rob.turn(180,Rover.turnside.ccw,1,Rover.axis.center);
                    rob.linear.setPower(1);
                    sleep(500);
                    rob.linear.setPower(0);

                case RIGHT:
                    rob.turn(35,Rover.turnside.cw,1,Rover.axis.center);
                    rob.driveTrainEncoderMovement(0.5,35.4*rotationperinch,5,200,Rover.movements.forward);

                    rob.driveTrainEncoderMovement(0.5,35.4*rotationperinch,5,200,Rover.movements.backward);

                    rob.turn(80,Rover.turnside.ccw,1,Rover.axis.center);
                    rob.driveTrainEncoderMovement(0.75,47.8*rotationperinch,5,200,Rover.movements.backward);

                    rob.turn(90,Rover.turnside.cw,1,Rover.axis.center);
                    rob.driveTrainEncoderMovement(0.75,70.8*rotationperinch,5,200,Rover.movements.forward);
                    rob.arm.setPower(0.3);
                    sleep(500);
                    rob.arm.setPower(0);

                    rob.driveTrainEncoderMovement(0.75,123.8*rotationperinch,5,200,Rover.movements.backward);

                    rob.turn(180,Rover.turnside.ccw,1,Rover.axis.center);
                    rob.linear.setPower(1);
                    sleep(500);
                    rob.linear.setPower(0);


            }
        }
    }
}
