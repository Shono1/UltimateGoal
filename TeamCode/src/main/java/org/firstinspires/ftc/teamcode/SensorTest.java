package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

@TeleOp
@Disabled
public class SensorTest extends OpMode {
    private DigitalChannel trig;
    private DigitalChannel echo;
    private boolean received = true;
    private long t = 0;
    @Override
    public void init() {
        trig = hardwareMap.get(DigitalChannel.class, "trig");
        echo = hardwareMap.get(DigitalChannel.class, "echo");
        trig.setMode(DigitalChannel.Mode.OUTPUT);
        echo.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {
        if(received) {
            trig.setState(true);
            received = false;
            long t = System.nanoTime();
        }
        if(echo.getState()) {
            received = true;
            telemetry.addData("time", System.nanoTime() - t);
            telemetry.update();
        }
    }
}
