package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
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
@Config
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
    public static double psVelo = 51;
    public static double HGVelo = 54;




    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(STARTING_POS);
        initVuforia();
        initTfod();
        int stackSize = 0; // 0 A, 1 B, 4 C

        Trajectory startingOffset = drive.trajectoryBuilder(STARTING_POS)
                .forward(12)
                .build();

        Trajectory toWobbleDeliveryBranch = drive.trajectoryBuilder(startingOffset.end())
                .strafeTo(new Vector2d(-2, -60))
                .addTemporalMarker(0, () -> {
                    drive.lowerWobble();
                })
                .addTemporalMarker(1.6, () -> {
                    drive.stopWobble();
                })
                .build();

        Trajectory toShoot = drive.trajectoryBuilder(toWobbleDeliveryBranch.end())
                .addTemporalMarker(0, () -> {
                    drive.spinFlywheel(HGVelo); // Set to top goal rpm
                })
                .back(24)
                .build();

        Trajectory wobbleDeliveryTraj1 = drive.trajectoryBuilder(toShoot.end())
                .lineToSplineHeading(new Pose2d(24, -52, Math.toRadians(-90)))
                .build();

        if(tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }

        while(!isStarted()) {
            drive.grab.setPower(-0.5);
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

                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) { // Single Stack / 2nd box
                            wobbleDeliveryTraj1 = drive.trajectoryBuilder(toShoot.end())
                                    .lineToSplineHeading(new Pose2d(24, -36, 0))
                                    .build();
                        }
                        else if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                            wobbleDeliveryTraj1 = drive.trajectoryBuilder(toShoot.end()) // Quad / 3rd
                                    .lineToSplineHeading(new Pose2d(63, -52, Math.toRadians(-90)))
                                    .build();
                        }
                    }
                    telemetry.update();


                }
            }
        }

        waitForStart();
        drive.followTrajectory(startingOffset);
        drive.followTrajectory(toWobbleDeliveryBranch); // Drive forward to near front of delivery zones
        drive.followTrajectory(toShoot);


        sleep(1500);
        for(int i = 0; i < 3; i++) { // Fire thrice
            drive.fire();
            sleep(700);
            drive.rest();
            sleep(500);
        }

        drive.spinFlywheel(0);
        drive.followTrajectory(wobbleDeliveryTraj1); // Drive to the correct drop zone
        drive.grab.setPower(0.5); // Release wobble goal from gripper
        sleep(200);
        drive.grab.setPower(0);
        Trajectory moveBaack = drive.trajectoryBuilder(wobbleDeliveryTraj1.end()).back(12).build();
        drive.followTrajectory(moveBaack);

        Trajectory park = drive.trajectoryBuilder(moveBaack.end())
                .lineToLinearHeading(new Pose2d(6, -30, 90))
                .build();

        drive.followTrajectory(park);

        Trajectory secondWobbleTrajOffset = drive.trajectoryBuilder(wobbleDeliveryTraj1.end())
                .splineToLinearHeading(new Pose2d(-12, -8, Math.toRadians(180)),0)
                .build();

//       // TODO: Powershot heading
//        double psHeading1 = 16; // Degrees; Angle to first powershot
//        Trajectory toPS = drive.trajectoryBuilder(wobbleDeliveryTraj1.end())
//                .splineToLinearHeading(new Pose2d(-6, -4, Math.toRadians(-90)), 0)
//                .addTemporalMarker(0, () -> {
//                    drive.spinFlywheel(psVelo);
//                }).build();
////
//        drive.followTrajectory(toPS); // Drive to powershot shooting spot
////
//        drive.fire();
//        sleep(200);
//        drive.rest();
//
//        drive.followTrajectory(drive.trajectoryBuilder(toPS.end())
//                .lineToLinearHeading(new Pose2d(-6, -12, Math.toRadians(-90)))
//                .build());
//
//        drive.fire();
//        sleep(200);
//        drive.rest();
//
//        drive.followTrajectory(drive.trajectoryBuilder(toPS.end())
//                .lineToLinearHeading(new Pose2d(-6, -20, Math.toRadians(-90)))
//                .build());
//
//        drive.fire();
//        sleep(200);
//        drive.rest();


//
//        // TODO: Second and third PS
//        Trajectory lastPSHeading;
//        drive.spinFlywheel(0);

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
        PoseStorage.imu = drive.getImu();
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
