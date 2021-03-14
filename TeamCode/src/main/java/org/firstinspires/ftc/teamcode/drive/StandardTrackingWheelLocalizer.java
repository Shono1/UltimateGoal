package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.LayoutRes;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 3.3411586; // in; distance between the left and right wheels
    public static double LATEEAL_Y_OFFSET = -0.1895;
    public static double FORWARD_OFFSET = -3.6255; // in; offset of the lateral wheel
    public static double FORWARD_Y_OFFSET = -1.3025; // in; lateral offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    /* Lines 37-38 in StandardTrackingWheelLocalizer.java */
    public static double X_MULTIPLIER = 0.97743679475334049606961272792313; // Multiplier in the X direction
    // 94.5 / 96.7826541825818
    // 94.5 / 96.74290006716065
    // 94.5 / 96.51919312046846
    public static double Y_MULTIPLIER = 0.9673518; // Multiplier in the Y direction
    // 94.5 / 89.60948629031263
    // 94.5 /
    // 94.5 /

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(2.838, LATERAL_DISTANCE / 2 + LATEEAL_Y_OFFSET, 0), // left
                new Pose2d(2.838, -LATERAL_DISTANCE / 2 + LATEEAL_Y_OFFSET, 0), // right
                new Pose2d(FORWARD_OFFSET, FORWARD_Y_OFFSET, Math.toRadians(90)) // front // DONE: y = Y_OFFSET!!! Original code will not work until y does not equal zero
        ));

//        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
//        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "wobble"));


        //frontEncoder.setDirection(Encoder.Direction.REVERSE);


        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition() * Y_MULTIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())
        );
    }
}
