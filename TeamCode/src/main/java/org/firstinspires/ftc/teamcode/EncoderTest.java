package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp
@Disabled
public class EncoderTest extends OpMode {
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    @Override
    public void init() {
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "wobble"));
    }

    @Override
    public void loop() {
        telemetry.addData("left encoder", leftEncoder.getCurrentPosition());
        telemetry.addData("right encoder", rightEncoder.getCurrentPosition());
        telemetry.addData("front encoder", frontEncoder.getCurrentPosition());
        telemetry.update();
    }
}
