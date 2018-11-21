package org.firstinspires.ftc.teamcode.old;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Central;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Blue 1", group = "Autonomous")

public class autonom extends CentralOld {

    team side = team.blue1;
    public static final String TAG = "Vuforia VuMark Sample";
    public ElapsedTime runtime = new ElapsedTime();
    public static int position = 0;


    VuforiaLocalizer vuforia;
    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        CentralClass(setupType.blue1, team.blue1);

        waitForStart();
        runtime.reset();



        while (opModeIsActive()) {

            encoderMovement(0.5, 6, 10, 100, movements.relicIn, motorFR);

                    break;
        }


    }

}
