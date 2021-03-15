package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AssetsTrajectoryManager;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.List;

@Autonomous
public class Auto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AWgzIBj/////AAABme+Omp6UckUCnzAYp16Mjcp/UHXxyuMuc910tm6lTjH/lwf9zbW+riaHA8Q5S0bbbmIV01s" +
                    "inje4/ovFpicpMViTFEw74Z7FRL5Ow8e7HnKwFv3RGkFHjCOgzKSjj3qhQKtkLm4Ua4KiPvp9WmK90d" +
                    "21+saouunyOcZjO7g/WEB36/kHmll4/n13WU9vtYdz1ojtl6UpJpH/MYTV6kcOZYSRzxXqP4Aecr5nm" +
                    "DBiukBJGmtMSFWr8dZmzsivejzNq99aQ9ny7o85LT9iEbxagyEZ0xVPxJabcv3mXVPh9Q3MTtiFiQ+S" +
                    "7xaeIct9lfKK+BkH1es3kQ8V9o+nEYM/QBB6HBHdeH2oqaBwBZhVJN8h";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final Pose2d STARTING_POS = new Pose2d(-63, -48, Math.toRadians(-90));
    private static Pose2d A_Pos = new Pose2d(12, -48, Math.toRadians(-90));
    private static Pose2d B_Pos = new Pose2d(36, -48, Math.toRadians(90));
    private static Pose2d C_Pos = new Pose2d(60, -48, Math.toRadians(-90));
    private final String LABELB = "B";
    private final String LABELC = "C";
    private static Pose2d powerShotPos = new Pose2d();
    private static double psVelo = 46;




    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(STARTING_POS);
        initVuforia();
        initTfod();
        int stackSize = 0; // 0 A, 1 B, 4 C

        Trajectory toWobbleDeliveryBranch = drive.trajectoryBuilder(STARTING_POS)
                .strafeTo(new Vector2d(-12, -48))
                .addTemporalMarker(1, () -> {
                    drive.lowerWobble();
                })
                .build();
        Trajectory wobbleDeliveryTraj1 = drive.trajectoryBuilder(toWobbleDeliveryBranch.end())
                .lineToSplineHeading(new Pose2d(12, -48, Math.toRadians(-90)))
                .build();

        while(!isStarted()) {

            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        if (recognition.getLabel().equals(LABELB)) {
                            wobbleDeliveryTraj1 = drive.trajectoryBuilder(toWobbleDeliveryBranch.end())
                                    .lineToSplineHeading(new Pose2d(36, -48, Math.toRadians(90)))
                                    .build();
                        }
                        else if (recognition.getLabel().equals(LABELC)) {
                            wobbleDeliveryTraj1 = drive.trajectoryBuilder(toWobbleDeliveryBranch.end())
                                    .lineToSplineHeading(new Pose2d(60, -48, Math.toRadians(-90)))
                                    .build();
                        }
                    }
                    telemetry.update();


                }
            }
        }

        waitForStart();
        drive.followTrajectory(toWobbleDeliveryBranch); // Drive forward to near front of delivery zones
        drive.followTrajectory(wobbleDeliveryTraj1); // Drive to the correct drop zone
        drive.dropWobble(); // Release wobble goal from gripper
        // TODO: Powershot heading
        double psHeading1 = 16; // Degrees; Angle to first powershot
        Trajectory toPS = drive.trajectoryBuilder(wobbleDeliveryTraj1.end())
                .splineToLinearHeading(new Pose2d(-6, 35, Math.toRadians(psHeading1)), 0)
                .addTemporalMarker(1, () -> {
                    drive.spinFlywheel(psVelo);
                }).build();

        drive.followTrajectory(toPS); // Drive to powershot shooting spot

        drive.fire();
        sleep(200);
        drive.rest();

        // TODO: Second and third PS
        Trajectory lastPSHeading;
        drive.spinFlywheel(0);

//        Trajectory grabWobbleTwo = drive.trajectoryBuilder(lastPSHeading.end())
//                .lineToLinearHeading(new Pose2d(-38, -24, Math.toRadians(180)))
//                .build();
//        drive.followTrajectory(grabWobbleTwo);
//        // TODO: Do the actual grabby
//
//        Trajectory deliverWobbleTwo = drive.trajectoryBuilder(grabWobbleTwo.end()) // TODO: This line
//                .lineTo()
//                .build();
//        drive.followTrajectory(deliverWobbleTwo);
//        drive.dropWobble();
//
//        Trajectory park = drive.trajectoryBuilder(deliverWobbleTwo.end())
//                .splineToLinearHeading(new Pose2d(12, -30, 0), 0)
//                .build();
//        drive.followTrajectory(park);
        PoseStorage.currectPose = drive.getPoseEstimate();
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        // parameters.cameraDirection = CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
