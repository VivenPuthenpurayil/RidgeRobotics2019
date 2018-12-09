
package org.firstinspires.ftc.teamcode.Autonomous;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.ClassFactory;
        import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
        import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
        import org.firstinspires.ftc.teamcode.Control.Rover;

        import java.util.ArrayList;
        import java.util.List;

        import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
        import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
        import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
@Autonomous(name="SamplingTest", group ="Smart")
public class SamplingTest extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        setup(runtime, Rover.setupType.autonomous);
        if(opModeIsActive()) {
            samplingNarrow();
        }
    }
}


