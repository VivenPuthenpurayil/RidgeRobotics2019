package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Sample Order Detector", group = "Smart")

public class SampleOrderDetector extends LinearOpMode {
    private SamplingOrderDetector Viven;

    public void runOpMode() throws InterruptedException {
        Viven = new SamplingOrderDetector();
        Viven.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        Viven.useDefaults();

        Viven.downscale = 0.4;

        Viven.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

        Viven.maxAreaScorer.weight = 0.005;

        Viven.ratioScorer.weight = 5;
        Viven.ratioScorer.perfectRatio = 1.0;

        Viven.enable();

    }
}