package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

// TODO: !!!Aimbot salvo is not finished!!! OPMode WILL NOT WORK !!!
// StopRelease
@Config
@TeleOp
public class FieldCentricDriveWithVuforia extends OpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight, intake, wobble;
    private DcMotorEx flywheel;
    private Servo fire;
    private CRServo grab;
    private BNO055IMU imu;
    private FtcDashboard dashboard;
    public static double p = 2;
    public static double i = 1;
    public static double d = -1;
    public static double f = 0;

    public static double velocity = 56; // rpm

    public static double firePos = 0;
    public static double restPos = 0.2;

    private long fireTime;
    private long restTime;
    private static int salvoWait = 200; // ms
    private static int threshhold = 30; // ticks / sec
    private boolean salvo = false;
    private int salvoState = 0;
    private int grabberMode = 0;

    private static double maxThetaDelta = 4; // Degrees
    private boolean correctAngle = false;
    private double targetTheta;
    private double aimbotSalvoTime;

    private static final String VUFORIA_KEY =
            "AWgzIBj/////AAABme+Omp6UckUCnzAYp16Mjcp/UHXxyuMuc910tm6lTjH/lwf9zbW+riaHA8Q5S0bbbmIV01s" +
                    "inje4/ovFpicpMViTFEw74Z7FRL5Ow8e7HnKwFv3RGkFHjCOgzKSjj3qhQKtkLm4Ua4KiPvp9WmK90d" +
                    "21+saouunyOcZjO7g/WEB36/kHmll4/n13WU9vtYdz1ojtl6UpJpH/MYTV6kcOZYSRzxXqP4Aecr5nm" +
                    "DBiukBJGmtMSFWr8dZmzsivejzNq99aQ9ny7o85LT9iEbxagyEZ0xVPxJabcv3mXVPh9Q3MTtiFiQ+S" +
                    "7xaeIct9lfKK+BkH1es3kQ8V9o+nEYM/QBB6HBHdeH2oqaBwBZhVJN8h";

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    List<VuforiaTrackable> allTrackables;

    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        flywheel = hardwareMap.get(DcMotorEx.class, "fly");
        intake = hardwareMap.get(DcMotor.class, "intake");
        wobble = hardwareMap.get(DcMotor.class, "wobble");

        fire = hardwareMap.get(Servo.class, "fire");
        grab = hardwareMap.crservo.get("grab");


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, -d, 0));

        dashboard = FtcDashboard.getInstance();
        initVuforia();
    }

    private void fire() {
        fire.setPosition(firePos);
        fireTime = System.currentTimeMillis();
    }

    private void rest() {
        fire.setPosition(restPos);
        restTime = System.currentTimeMillis();
    }


    public void loop()  {
        flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d = d;
        double gyroTheta = (imu.getAngularOrientation().firstAngle + (2 * Math.PI)) % (2 * Math.PI);
        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double joyTheta = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        joyTheta += gyroTheta;
        setVelos(magnitude * Math.sin(joyTheta), -magnitude * Math.cos(joyTheta), 0.8 * gamepad1.right_stick_x);

        telemetry.addData("PIDF", flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());

        if(gamepad1.a) {
            intake.setPower(1);
        }
        else if(gamepad1.b) {
            intake.setPower(-1);
        }
        else {
            intake.setPower(0);
        }

        if(gamepad1.right_bumper) {
            flywheel.setVelocity(28 * velocity);
        }
        else if(gamepad1.left_bumper) {
            flywheel.setPower(1);
        }
        else {
            flywheel.setVelocity(0);
        }

        if(gamepad1.y) {
            salvo = true;
            if(salvoState == 0 && (flywheel.getVelocity() > 28 * velocity - threshhold && flywheel.getVelocity() < 28 * velocity - threshhold) ) { // TODO: This is broken
                fire();
                salvoState = 1;
            }
            else if(salvoState == 1 && System.currentTimeMillis() > fireTime + salvoWait) {
                rest();
                salvoState = 2;
            }
            else if(salvoState == 2 && System.currentTimeMillis() > restTime + salvoWait) {
                salvoState = 0;
            }

        }
        else {
            salvoState = 0;
            salvo = false;
        }

        if(gamepad1.x && !salvo) {
//            fire.setPosition(firePos);
//            fireTime = System.currentTimeMillis();
            fire();
        }
        else if(System.currentTimeMillis() > fireTime + 200 && !salvo) {
            fire.setPosition(restPos);
        }




        if(gamepad1.right_trigger > 0) {
            wobble.setPower(gamepad1.right_trigger);
        }
        else if(gamepad1.left_trigger > 0) {
            wobble.setPower(-gamepad1.left_trigger);
        }
        else {
            wobble.setPower(0);
        }

        if (gamepad1.dpad_left) {
            grab.setPower(0.5);
        }
        else if(gamepad1.dpad_right) {
            grab.setPower(-0.5);
        }
        else {
            grab.setPower(0);
        }

        TelemetryPacket velo = new TelemetryPacket();
        velo.put("velocity", flywheel.getVelocity());
        velo.put("delta", velocity * 28 - flywheel.getVelocity());
        velo.put("pidf", flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
        dashboard.sendTelemetryPacket(velo);

        double[] location = vuforiaLoop();
        if(correctAngle) {
            fixHeading(location[2] - 90, targetTheta);
        }

        if(gamepad1.back) {
            aimbot(location);
        }

        velocity = getFlywheelVelo(-(location[0] - 72));


    }

    private void aimbot(double[] location) { // Xpos, Ypos, Zrot
        double correctedX = -(location[0] - 72); // Distance back from target
        double correctedY = (location[1] + 36); // Distance from centreline of vumark; pos to right
        double correctedZAngle = location[2] - 90;

        double hypot = Math.hypot(location[0], location[1]);
        targetTheta = Math.atan2(correctedY, correctedX);

        double thetaDelta = correctedZAngle - targetTheta;

        // Every 12in, plus 0.5 power
        double targetVelo = (0.5 / 12) * correctedX + 54.375;

        if(Math.abs(thetaDelta) > maxThetaDelta) {
            aimbotSalvoTime = System.currentTimeMillis();
            correctAngle = false;
            flywheel.setVelocity(targetVelo * 28);
            // TODO: Fire control
        }
        else {
            correctAngle = true;
        }

        telemetry.addData("x distance", correctedX);
        telemetry.addData("y distance", correctedY);
        telemetry.addData("hypot", hypot);
        telemetry.update();
    }

    private void fixHeading(double currentTheta, double targetTheta) {
        double correction = currentTheta - targetTheta;
        if(Math.abs(correction) > maxThetaDelta) {
            correctAngle = false;
            salvo = true;
            aimbotSalvoTime = System.currentTimeMillis();
            return;
        }
        double correctionVelo = (correction / 90);
        if(correctionVelo > 0) {
            correctionVelo += 0.25;
        }
        else {
            correctionVelo -= 0.25;
        }

        setVelos(0, 0, correctionVelo);

    }

    private double getFlywheelVelo(double correctedX) {
        return (0.5 / 12) * correctedX + 54.375;
    }

    private void setVelos(double y, double x, double r) {
        frontLeft.setPower(y + x + r);
        backLeft.setPower(y - x + r);
        frontRight.setPower(y - x - r);
        backRight.setPower(y + x - r);
    }

    private void initVuforia() {
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.


        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Camera is 15 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;     // eg: Camera 8.5in off center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        targetsUltimateGoal.activate();
        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:
    }

    private double[] vuforiaLoop() {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        double[] ret = new double[3];
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            ret[0] = translation.get(0);
            ret[1] = translation.get(1);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            ret[2] = rotation.thirdAngle;
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
        return(ret);
    }

}



