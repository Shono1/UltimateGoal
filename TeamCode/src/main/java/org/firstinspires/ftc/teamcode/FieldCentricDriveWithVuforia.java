package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.MyTwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Config
@TeleOp(name = "FCD Tele")
public class FieldCentricDriveWithVuforia extends LinearOpMode {
    private static final String TAG = "FCD With Vuforia";
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

    public static double velocity = 54.5; // rpm

    public static double firePos = 0;
    public static double restPos = 0.2;

    private long fireTime;
    private long restTime;

    private Pose2d STARTING_POSE;
    private MyTwoWheelTrackingLocalizer local;

    private final Vector2d GOAL_POS = new Vector2d(72, -36);
    private final  Vector2d PS_LEFT = new Vector2d(72, -8);
    private final Vector2d PS_MID = new Vector2d(72, -16);
    private final Vector2d PS_RIGHT = new Vector2d(72, -24);


    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        flywheel = hardwareMap.get(DcMotorEx.class, "fly");
        intake = hardwareMap.get(DcMotor.class, "intake");
        wobble = hardwareMap.get(DcMotor.class, "wobble");

        fire = hardwareMap.get(Servo.class, "fire");
        grab = hardwareMap.crservo.get("grab");
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.mode = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled = false;
//        imu.initialize(parameters);
        imu = PoseStorage.imu;
        STARTING_POSE = PoseStorage.currectPose;
        local = new MyTwoWheelTrackingLocalizer(hardwareMap, imu);
        local.setPoseEstimate(STARTING_POSE);


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, -d, 0));

        dashboard = FtcDashboard.getInstance();
        waitForStart();
        while(opModeIsActive()) {
            run();
        }
    }

    private void fire() {
        fire.setPosition(firePos);
        fireTime = System.currentTimeMillis();
    }

    private void rest() {
        fire.setPosition(restPos);
        restTime = System.currentTimeMillis();
    }


    public void run()  {
        flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d = d;
        double gyroTheta = (imu.getAngularOrientation().firstAngle + (2 * Math.PI)) % (2 * Math.PI) - Math.PI;
        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double joyTheta = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        joyTheta += gyroTheta;
        setVelos(magnitude * Math.sin(joyTheta), -magnitude * Math.cos(joyTheta), 0.8 * gamepad1.right_stick_x);

        Pose2d estimate = local.getPoseEstimate();
        local.update();
        telemetry.addData("x", estimate.getX());
        telemetry.addData("y", estimate.getY());
        telemetry.addData("heading", estimate.getHeading());
        telemetry.update();

        if(gamepad2.a) {
            intake.setPower(1);
        }
        else if(gamepad2.b) {
            intake.setPower(-1);
        }
        else {
            intake.setPower(0);
        }

        if(gamepad2.right_bumper) {
            flywheel.setVelocity(28 * velocity);
        }
        else {
            flywheel.setVelocity(0);
        }

        if(gamepad2.x) {
//            fire.setPosition(firePos);
//            fireTime = System.currentTimeMillis();
            fire();
        }
        else if(System.currentTimeMillis() > fireTime + 200) {
            fire.setPosition(restPos);
        }


        if(gamepad2.dpad_up) {
            wobble.setPower(-0.5);
        }
        else if(gamepad2.dpad_down) {
            wobble.setPower(0.5);
        }
        else {
            wobble.setPower(0);
        }

        if (gamepad2.dpad_left) {
            grab.setPower(-0.5);
        }
        else if(gamepad2.dpad_right) {
            grab.setPower(0.5);
        }
        else {
            grab.setPower(0);
        }
        if(gamepad1.x) {
            alignAndShoot(estimate, GOAL_POS, 3, this::getFlywheelVelo);
        }
        if(gamepad1.y) {
            alignAndShoot(estimate, PS_LEFT, 1, this::getPSVelo);
            alignAndShoot(estimate, PS_MID, 1, this::getPSVelo);
            alignAndShoot(estimate, PS_RIGHT, 1, this::getPSVelo);
        }

        if(gamepad1.back) {
            recalibratePose();
        }

        TelemetryPacket velo = new TelemetryPacket();
        velo.put("velocity", flywheel.getVelocity());
        velo.put("delta", velocity * 28 - flywheel.getVelocity());
        // velo.put("pidf", flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
        dashboard.sendTelemetryPacket(velo);
        // drive.update();


    }


    private double getFlywheelVelo(double dist) {
        // return (7.7857 / 12.0) * dist * 28;
        return 11.9 * Math.log(dist) * 28;
    }

    private double getPSVelo(double dist) {
        return 11.5 * Math.log(dist) * 28;
    }

    private void setVelos(double y, double x, double r) {
        frontLeft.setPower(y + x + r);
        backLeft.setPower(y - x + r);
        frontRight.setPower(y - x - r);
        backRight.setPower(y + x - r);

        // drive.setMotorPowers(-(y + x + r), -(y - x + r), -(y + x - r), -(y - x - r));
    }

    private void alignAndFire(Pose2d estimate) {
        double xDiff = GOAL_POS.getX() - estimate.getX();
        double yDiff = GOAL_POS.getY() - estimate.getY();
        double distance = Math.hypot(xDiff, yDiff);
        double theta = Math.atan2(yDiff, xDiff);
        Orientation ao = imu.getAngularOrientation();
        telemetry.addData("theta", theta);
        telemetry.addData("imu", ao.firstAngle);
        telemetry.addData("x diff", xDiff);
        telemetry.update();
        double fwVelo = getFlywheelVelo(distance);
        flywheel.setVelocity(fwVelo);
        while(Math.abs(ao.firstAngle - theta) > 0.07 && opModeIsActive()) {
            setVelos(0,0,0.2 * Math.abs(ao.firstAngle - theta) / (ao.firstAngle - theta));
            ao = imu.getAngularOrientation();
            telemetry.addData("aaaaaaa", "lgbtlajojnfkajdfnkasfnd");
            idle();
        }

        setVelos(0, 0, 0);
        for(int i = 0; i < 3; i++) {
            while (Math.abs(fwVelo - flywheel.getVelocity()) > 10) {
                idle();
            }
            Log.d(TAG, "Dist: " + distance + "  Pow: " + flywheel.getVelocity() + "  Target: " + fwVelo);
            fire();
            while(System.currentTimeMillis() < fireTime + 300) {
                idle();
            }
            rest();
            while(System.currentTimeMillis() < fireTime + 500) {
                idle();
            }
        }
    }

    private void powerShots(Pose2d estimate) {
        double xDiff = PS_LEFT.getX() - estimate.getX();
        double yDiff = PS_LEFT.getY() - estimate.getY();
        double distance = Math.hypot(xDiff, yDiff);
        double theta = Math.atan2(yDiff, xDiff);
        Orientation ao = imu.getAngularOrientation();
        telemetry.addData("theta", theta);
        telemetry.addData("imu", ao.firstAngle);
        telemetry.addData("x diff", xDiff);
        telemetry.update();
        double fwVelo = getPSVelo(distance);
        flywheel.setVelocity(fwVelo);
        while(Math.abs(ao.firstAngle - theta) > 0.07 && opModeIsActive()) {
            setVelos(0,0,0.2 * Math.abs(ao.firstAngle - theta) / (ao.firstAngle - theta));
            ao = imu.getAngularOrientation();
            telemetry.addData("aaaaaaa", "lgbtlajojnfkajdfnkasfnd");
            idle();
        }

        setVelos(0, 0, 0);
        for(int i = 0; i < 3; i++) {
            while (Math.abs(fwVelo - flywheel.getVelocity()) > 10) {
                idle();
            }
            Log.d(TAG, "Dist: " + distance + "  Pow: " + flywheel.getVelocity() + "  Target: " + fwVelo);
            fire();
            while(System.currentTimeMillis() < fireTime + 300) {
                idle();
            }
            rest();
            while(System.currentTimeMillis() < fireTime + 500) {
                idle();
            }
        }
    }

    private void alignAndShoot(Pose2d estimate, Vector2d target, int n, Function<Double, Double> velo) {
        double xDiff = target.getX() - estimate.getX();
        double yDiff = target.getY() - estimate.getY();
        double distance = Math.hypot(xDiff, yDiff);
        double theta = Math.atan2(yDiff, xDiff);
        Orientation ao = imu.getAngularOrientation();
        telemetry.addData("theta", theta);
        telemetry.addData("imu", ao.firstAngle);
        telemetry.addData("x diff", xDiff);
        telemetry.update();
        double fwVelo = velo.apply(distance);
        flywheel.setVelocity(fwVelo);
        while(Math.abs(ao.firstAngle - theta) > 0.02 && opModeIsActive()) {
            setVelos(0,0,0.2 * Math.abs(ao.firstAngle - theta) / (ao.firstAngle - theta));
            ao = imu.getAngularOrientation();
            telemetry.addData("aaaaaaa", "lgbtlajojnfkajdfnkasfnd");
            idle();
        }

        setVelos(0, 0, 0);
        for(int i = 0; i < n; i++) {
            while (Math.abs(fwVelo - flywheel.getVelocity()) > 10) {
                idle();
            }
            Log.d(TAG, "Dist: " + distance + "  Pow: " + flywheel.getVelocity() + "  Target: " + fwVelo);
            fire();
            while(System.currentTimeMillis() < fireTime + 300) {
                idle();
            }
            rest();
            while(System.currentTimeMillis() < fireTime + 500) {
                idle();
            }
        }
    }

    private void recalibratePose() {
        local.setPoseEstimate(new Pose2d(-63, 22, imu.getAngularOrientation().firstAngle));
        // Starting y + 70
        // Starting x
    }
}