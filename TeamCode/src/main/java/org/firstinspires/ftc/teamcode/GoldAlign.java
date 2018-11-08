package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Gold Align", group = "Smart")

public class GoldAlign extends LinearOpMode {
    private GoldAlignDetector Viven;

    public void runOpMode() throws InterruptedException {
        Viven = new GoldAlignDetector();
        Viven.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        Viven.useDefaults();

        Viven.alignSize = 500;
        Viven.alignPosOffset = 0;
        Viven.downscale = 0.4;

        Viven.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

        Viven.maxAreaScorer.weight = 0.005;

        Viven.ratioScorer.weight = 5;
        Viven.ratioScorer.perfectRatio = 1.0;

        Viven.enable();

    }
}