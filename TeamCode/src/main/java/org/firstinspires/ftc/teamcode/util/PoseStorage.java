package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class PoseStorage {
    public static Pose2d currectPose = new Pose2d();
    public static BNO055IMU imu;
}
