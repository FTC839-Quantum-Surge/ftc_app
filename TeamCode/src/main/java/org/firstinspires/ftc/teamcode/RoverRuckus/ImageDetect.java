package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class ImageDetect
{
    public class GoldPositionInfo
    {
        public double  angle = 0.0;
        public int     count = 0;
        public int     position = -1;

        public boolean found() { return count > 0; }

        public GoldPositionInfo()
        {
        }

        public GoldPositionInfo( GoldPositionInfo src )
        {
            count    = src.count;
            angle    = src.angle;
            position = src.position;
        }
    };

    private static final String TFOD_MODEL_ASSET     = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL   = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AQbZ+sL/////AAABmSa58vHJC05MmNbUKKXaK4op5HPfqwUNiA6+OZ9cpccMRQZteFBlFOeAVRJqFoDm6lcSTSnMsEyzjXf2sEkSDgLTYCKEPxIckdKYrogIb7FACCFehqKMLOCToCEadaQQ3SVrU/G4oqtKYRye1yxIa2NJee6pCMEYsJK4enV6VNNX0LTgJDuvr43h99X/JraziB1mNyKJzPvYTti40x9cAfmlq4b9Qz5vAW/a7wzOsnm205gBcKm3Zigt2J8eB22OMfPgGrdUiuQ1uoNYgDA+qdRWsnxDo85IjRdl2QXZWN3/85S9Eqh3MtVgL3DbG3ZEG/u3wsRpIENSJ4udsEbNVlmF4Gx94YATrBP8WQ9E0iGO";

    private GoldPositionInfo m_info      = new GoldPositionInfo();
    private Telemetry        m_telemetry = null;
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public void Initialize( HardwareMap hardwareMap, Telemetry telemetry )
    {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        m_telemetry = telemetry;

        initVuforia( hardwareMap );

        if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            initTfod( hardwareMap );
        }
        else
        {
            m_telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            return;
        }

        if (tfod != null)
            tfod.activate();
    }

    public  GoldPositionInfo GetGoldPos()
    {
        return new GoldPositionInfo( m_info );
    }

    public GoldPositionInfo DetectMinierals()
    {
        if (tfod != null)
        {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null)
            {
                GoldPositionInfo localInfo = new GoldPositionInfo();

               // telemetry.addData("# Object Detected", updatedRecognitions.size());

                int goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;

                for (Recognition recognition : updatedRecognitions)
                {

                    double angle = recognition.estimateAngleToObject( AngleUnit.DEGREES );
                    //telemetry.addData("item   :", recognition.getLabel() );
                    //telemetry.addData("  Angle:",  angle );

                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                    {
                        localInfo.count ++;
                        localInfo.angle = angle;

                        goldMineralX = (int) recognition.getLeft();
                    }
                    else if (silverMineral1X == -1)
                    {
                        silverMineral1X = (int) recognition.getLeft();
                    }
                    else
                    {
                        silverMineral2X = (int) recognition.getLeft();
                    }
                }

                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1)
                {
                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X)
                    {
                        localInfo.position = -1;
              //          telemetry.addData("Gold Mineral Position", "Left");
                    }
                    else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X)
                    {
                        localInfo.position = 1;
              //          telemetry.addData("Gold Mineral Position", "Right");
                    }
                    else
                    {
                        localInfo.position = 0;
              //          telemetry.addData("Gold Mineral Position", "Center");
                    }
                }

                m_info = new GoldPositionInfo( localInfo );

                return localInfo;
            }
        }
        return GetGoldPos();
    }

    public void Stop()
    {
        if (tfod != null)
        {
            tfod.shutdown();
        }
    }


    // Initialize the Vuforia localization engine.


    private void initVuforia( HardwareMap hardwareMap ) {

        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraDirection = CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

//     * Initialize the Tensor Flow Object Detection engine.
    private void initTfod( HardwareMap hardwareMap ) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}

