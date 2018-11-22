package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Central;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public abstract class AutonomousControl extends Central {
    /*just dont touch this part and we'll prb be ok????*/
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        private static final String VUFORIA_KEY = "AUaoObT/////AAABmZESOvJAgkXJk00eebGyewdT7a9NZK5YLL9rnvWS5jwOFcmnubSqY4E8gnBkljxMEsVfBteT2JE95kMUrCT379Ya4Inep4AQqT2IQRvFD5lTr2PYVIWp9c6oe2f1C9T8M1aco5W/O4kU1kOf0UGikrcSheCnor0nc2siDkbfT8s1YRRZVXl56xrCw7Po6PsU4tkZJ9F2tp9YyMmowziEIjbeLJ+V3C51kRnNyiMuF3ev0Einp5ioXgW82RJMPiJLiiKZZP9ARLYGLsNX+nOFFJXRrKywydpwcWwlyAaHzWpSJ9yWgAMwFY1iN4BBq8VaFInXY+T40/g/WCBxM7WLkWNNOe44921UcyFGgjqxf/T2";

        // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
        // We will define some constants and conversions here
        private static final float mmPerInch        = 25.4f;
        private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
        private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

        // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
        // Valid choices are:  BACK or FRONT
        private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

        private OpenGLMatrix lastLocation = null;
        private boolean targetVisible = false;

        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */
        VuforiaLocalizer vuforia;
    //lol

    public void deployLander() throws InterruptedException {
        while (!rob.deployingLimit.getState() && opModeIsActive()){
            rob.anyMovement(0.8, Rover.movements.rackDown, rob.rack);
        }
        rob.rack.setPower(0);
    }

    public void angleOfLander() throws InterruptedException {
        VectorF translation = lastLocation.getTranslation();
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

        // express the rotation of the robot in degrees.
        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

        telemetry.update();
        if (rotation.thirdAngle > 200 && rotation.thirdAngle < 205) {
            rob.driveTrainMovement(0.2, Rover.movements.cw);
        } else {
            rob.stopDrivetrain();
        }
    }

    public void centerPosition() throws InterruptedException {
        VectorF translation = lastLocation.getTranslation();
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

        // express the rotation of the robot in degrees.
        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

        telemetry.update();
        if (!((translation.get(0) > 50)&&(translation.get(0) < 55))) {
            rob.driveTrainMovement(0.2, Rover.movements.right);
        } else {
            rob.stopDrivetrain();
        }

    }

    public void leftPosition() throws InterruptedException {
        VectorF translation = lastLocation.getTranslation();
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

        // express the rotation of the robot in degrees.
        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

        telemetry.update();
        if (!((translation.get(0) > 30)&&(translation.get(0) < 35))) {
            rob.driveTrainMovement(0.2, Rover.movements.right);
        } else {
            rob.stopDrivetrain();
        }

    }

    public void rightPosition() throws InterruptedException {
        VectorF translation = lastLocation.getTranslation();
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

        // express the rotation of the robot in degrees.
        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

        telemetry.update();
        if (!((translation.get(0) > 70)&&(translation.get(0) < 75))) {
            rob.driveTrainMovement(0.2, Rover.movements.right);
        } else {
            rob.stopDrivetrain();
        }

    }

    public void knockingOffCenter() throws InterruptedException{
        rob.driveTrainEncoderMovement(0.2, 10,10, 2, Rover.movements.forward);
        //drop off the team marker stuff isn't configured/made
    }

    public void knockingOffLeft() throws InterruptedException{
        rob.driveTrainEncoderMovement(0.2, 5,10, 2, Rover.movements.forward);
        rob.driveTrainEncoderMovement(0.2, 5,10, 2, Rover.movements.right);
        //drop off the team marker stuff isn't configured/made
    }

    public void knockingOffRight() throws InterruptedException{
        rob.driveTrainEncoderMovement(0.2, 5,10, 2, Rover.movements.forward);
        rob.driveTrainEncoderMovement(0.2, 5,10, 2, Rover.movements.left);
        //drop off the team marker stuff isn't configured/made
    }

    public void goToCraterCraterSide() throws InterruptedException{
        rob.turn(45, Rover.turnside.ccw,.5, Rover.axis.back);
        rob.driveTrainEncoderMovement(0.5, 144, 20, 1, Rover.movements.backward);
    }

    public void goToCraterOtherSide() throws InterruptedException{
        rob.turn(45, Rover.turnside.cw,.5, Rover.axis.back);
        rob.driveTrainEncoderMovement(0.5, 144, 20, 1, Rover.movements.backward);
    }

    public void sampling () throws InterruptedException {
        SamplingOrderDetector VivenIsDumb = new SamplingOrderDetector();
        VivenIsDumb.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        VivenIsDumb.useDefaults();
        VivenIsDumb.downscale = 0.4;
        VivenIsDumb.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        VivenIsDumb.maxAreaScorer.weight = 0.005;
        VivenIsDumb.ratioScorer.weight = 5;
        VivenIsDumb.ratioScorer.perfectRatio = 1.0;
        VivenIsDumb.enable();

        switch(VivenIsDumb.getCurrentOrder()){
            case LEFT:
                leftPosition();
                knockingOffLeft();
            case CENTER:
                centerPosition();
                knockingOffCenter();
            case RIGHT:
                rightPosition();
                knockingOffRight();
            case UNKNOWN:
                sampling();
        }

    }

    


}
