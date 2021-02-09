package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class FieldCentricDrive extends OpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight, intake;
    private DcMotorEx flywheel;
    private Servo fire;
    private BNO055IMU imu;
    private FtcDashboard dashboard;
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static double velocity = 0.67;

    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        flywheel = hardwareMap.get(DcMotorEx.class, "fly");
        intake = hardwareMap.get(DcMotor.class, "intake");

        fire = hardwareMap.get(Servo.class, "fire");

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
    public void loop() {
        double theta = imu.getAngularOrientation().firstAngle;
        double fwd = gamepad1.left_stick_y * Math.cos(theta) + gamepad1.left_stick_x * Math.sin(theta);
        double strafe = -gamepad1.left_stick_y * Math.sin(theta) + gamepad1.left_stick_x * Math.cos(theta);
        setVelos(fwd, strafe, 0.4 * gamepad1.right_stick_x);

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
            flywheel.setVelocity(velocity);
        }
        else {
            flywheel.setPower(0);
        }

        if(gamepad1.x) {
            fire.setPosition(0.75);
            fireTime = System.currentTimeMillis();
        }
        else if(System.currentTimeMillis() > fireTime + 750) {
            fire.setPosition(0.92);
        }

        telemetry.addData("pos", fire.getPosition());
        telemetry.addData("pow", flywheel.getPower());
        telemetry.addData("vel", flywheel.getVelocity());
        telemetry.update();

    }

    private void setVelos(double y, double x, double r) {
        frontLeft.setPower(y + x + r);
        backLeft.setPower(y - x + r);
        frontRight.setPower(y - x - r);
        backRight.setPower(y + x - r);
    }
}



