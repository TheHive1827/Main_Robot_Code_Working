// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;


public final class Constants {

    // Swerve constants

    public static final class SwerveConstants {

        // Rotor IDs
        public static final int kLeftFrontRotorID = 1;
        public static final int kRightFrontRotorID = 3;
        public static final int kLeftRearRotorID = 7;
        public static final int kRightRearRotorID = 5;

        // Throttle IDs
        public static final int kLeftFrontThrottleID = 2;
        public static final int kRightFrontThrottleID = 4;
        public static final int kLeftRearThrottleID = 8;
        public static final int kRightRearThrottleID = 6;

        // Rotor encoder IDs
        public static final int kLeftFrontCANCoderID = 1;
        public static final int kRightFrontCANCoderID = 2;
        public static final int kLeftRearCANCoderID = 4;
        public static final int kRightRearCANCoderID = 3;

        // Rotor encoder & motor inversion
        public static final SensorDirectionValue kRotorEncoderDirection = SensorDirectionValue.CounterClockwise_Positive;
        public static final boolean kRotorMotorInversion = true;

        // IMU ID
        public static final int kImuID = 0;

        // Rotor encoder offsets
        public static final double kLeftFrontRotorOffset = -0.35254;
        public static final double kRightFrontRotorOffset = -0.33154;
        public static final double kLeftRearRotorOffset = -0.09765;
        public static final double kRightRearRotorOffset = -0.41650;

        public static final double LENGTH = Units.inchesToMeters(26.5);
        public static final double WIDTH = Units.inchesToMeters(22.5);

        // Swerve kinematics (order: left front, right front, left rear, right rear)
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
            new Translation2d(LENGTH/2, WIDTH/2),
            new Translation2d(LENGTH/2, -WIDTH/2),
            new Translation2d(-LENGTH/2, WIDTH/2),
            new Translation2d(-LENGTH/2, -WIDTH/2)
        );

        // Rotor PID constants
        public static final double kRotor_kP = 0.01;
        public static final double kRotor_kI = 0.0;
        public static final double kRotor_kD = 0.0;

        // elevator PID
        public static final double kElevatorP = 0.01;
        public static final double kElevatorI = 0.0;
        public static final double kElevatorD = 0.0;

        // Velocity & acceleration of swerve
        public static final double kMaxVelocityMetersPerSecond = 3.0;
        public static final double kMaxAccelerationMetersPerSecond = 3.0;

        // Wheel diameter
        // 輪徑
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); // wheel diameter
        
        // Throttle gear ratio
        // (number of turns it takes the motor to rotate the rotor one revolution)
        public static final double kThrottleGearRatio = 21.4285714286; 

        // Throttle velocity conversion constant
        // This value will be multiplied to the raw RPM of the throttle motor
        // and should convert it to meters per second
        // This is the general formula: 
        //     (1.0 / GEAR RATIO / 60.0_seconds) * WHEEL DIAMETER * Math.PI;
        public static final double kThrottleVelocityConversionFactor = 
            (1/kThrottleGearRatio/60)*kWheelDiameterMeters*Math.PI;

        public static final double kThrottlePositionConversionFactor = 
            (1/kThrottleGearRatio)*kWheelDiameterMeters*Math.PI;
        
        // Pathing PID constants 
        public static final double kPathingX_kP = 0.1;
        public static final double kPathingX_kI = 0.0;
        public static final double kPathingX_kD = 0.0;

        public static final double kPathingY_kP = 0.1;
        public static final double kPathingY_kI = 0.0;
        public static final double kPathingY_kD = 0.0;

        public static final double kPathingTheta_kP = 0.1;
        public static final double kPathingTheta_kI = 0.0;
        public static final double kPathingTheta_kD = 0.0;
    }
    
    // Voltage compensation
    public static final double kVoltageCompensation = 12.0;
    
    // Controller port
    public static final int kControllerPort = 0;

    public static final double kLongTimeoutMs = 100.;

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.05;
      }

      public static final class ElevatorConstants{
        public static final int ElevatorLeader = 10;

      }

      public static final class ArmConstants{
        public static final int armID = 12;
        public static final double armMAXWhen = 0.619048;
        public static final double armMinimum = 0.619048;
        public static final double MAX_DRIVING_VELOCITY_METERS_PER_SECOND = 5;
        public static final double MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 8;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 20;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 40;

        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED);
        public static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
      }
}
