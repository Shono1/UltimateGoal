package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class FieldCentricDrive extends OpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private BNO055IMU imu;
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");


    }

    public void loop() {
        frontLeft.setPower(gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x);
        backLeft.setPower(gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x);
        frontRight.setPower(gamepad1.left_stick_x - gamepad1.left_stick_y + gamepad1.right_stick_x);
        backRight.setPower(gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x);
    }
}
