package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp
@Disabled
public class FieldCentricDrive extends OpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight, intake, wobble;
    private DcMotorEx flywheel;
    private Servo fire;
    private CRServo grab;
    private BNO055IMU imu;
    private FtcDashboard dashboard;
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static double velocity = 56;

    public static double firePos = 0;
    public static double restPos = 0.2;

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


        dashboard = FtcDashboard.getInstance();


    }

    private long fireTime;
    private int grabberMode = 0;
    public void loop() {
        double gyroTheta = (imu.getAngularOrientation().firstAngle + (2 * Math.PI)) % (2 * Math.PI);
        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double joyTheta = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        joyTheta += gyroTheta;
        setVelos(magnitude * Math.sin(joyTheta), -magnitude * Math.cos(joyTheta), 0.8 * gamepad1.right_stick_x);

        // flywheel.setVelocityPIDFCoefficients(p, i, d, f);

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

        if(gamepad1.x) {
            fire.setPosition(firePos);
            fireTime = System.currentTimeMillis();
        }
        else if(System.currentTimeMillis() > fireTime + 200) {
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
    }

    private void setVelos(double y, double x, double r) {
        frontLeft.setPower(y + x + r);
        backLeft.setPower(y - x + r);
        frontRight.setPower(y - x - r);
        backRight.setPower(y + x - r);
    }
}



