// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveSubsystem;

public final class Constants
{
    public static final double CHASSIS_LENGTH_METERS = 0.62865;
    public static final double CHASSIS_WIDTH_METERS = 0.62865;
    public static final double CHASSIS_WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);

    public static final double CHASSIS_ANGLE_GEAR_RATIO = 21.42;
    public static final SwerveSubsystem.GearRatio CHASSIS_DRIVE_GEAR_RATIO = SwerveSubsystem.GearRatio.L1;

    public static final Pose2d CHASSIS_INITIAL_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0)); // TODO: Remember to use actual start pos

    public static final double CHASSIS_DRIVE_MOTOR_KS = 0.077618 / 12;
    public static final double CHASSIS_DRIVE_MOTOR_KV = 0.89851 / 12;
    public static final double CHASSIS_DRIVE_MOTOR_KA = 0.18239 / 12;

    public static final double CHASSIS_DRIVE_MOTOR_KP = 0.05;
    public static final double CHASSIS_DRIVE_MOTOR_KD = 0.0;

    public static final double CHASSIS_ANGLE_MOTOR_KP = 0.0001;
    public static final double CHASSIS_ANGLE_MOTOR_KD = 0.0;
    public static final double CHASSIS_ANGLE_MOTOR_KF = 0.00017855;

    public static final double CHASSIS_ANGLE_MOTOR_SMART_MOTION_MAX_VELOCITY = 5760;
    public static final double CHASSIS_ANGLE_MOTOR_SMART_MOTION_MAX_ACCEL = 10000;
    public static final double CHASSIS_ANGLE_MOTOR_SMART_MOTION_ALLOWABLE_ERROR = 0.01;

    public static final double CHASSIS_FRONT_LEFT_ANGLE_OFFSET = 0.0;
    public static final double CHASSIS_BACK_LEFT_ANGLE_OFFSET = 0.0;
    public static final double CHASSIS_BACK_RIGHT_ANGLE_OFFSET = 0.0;
    public static final double CHASSIS_FRONT_RIGHT_ANGLE_OFFSET = 0.0;

    public static final int CHASSIS_FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    public static final int CHASSIS_FRONT_LEFT_ANGLE_MOTOR_ID = 1;
    public static final int CHASSIS_FRONT_LEFT_CANCODER_ID = 1;

    public static final int CHASSIS_BACK_LEFT_DRIVE_MOTOR_ID = 2;
    public static final int CHASSIS_BACK_LEFT_ANGLE_MOTOR_ID = 2;
    public static final int CHASSIS_BACK_LEFT_CANCODER_ID = 2;

    public static final int CHASSIS_BACK_RIGHT_DRIVE_MOTOR_ID = 3;
    public static final int CHASSIS_BACK_RIGHT_ANGLE_MOTOR_ID = 3;
    public static final int CHASSIS_BACK_RIGHT_CANCODER_ID = 3;

    public static final int CHASSIS_FRONT_RIGHT_DRIVE_MOTOR_ID = 4;
    public static final int CHASSIS_FRONT_RIGHT_ANGLE_MOTOR_ID = 4;
    public static final int CHASSIS_FRONT_RIGHT_CANCODER_ID = 4;
}
