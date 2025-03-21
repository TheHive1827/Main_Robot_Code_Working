package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.DifferentialVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.fasterxml.jackson.annotation.JsonInclude.Include;
// import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
     //rotor & throttle
    private SparkMax mRotor;
    private SparkMax mThrottle;

    //  throttle encoder
    private RelativeEncoder mThrottleEncoder;

    // rotor encoder
    private CANcoder mRotorEncoder;

    // rotor PID controller
    private PIDController mRotorPID;

    private String m_name;

    public SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public SparkMaxConfig turningConfig = new SparkMaxConfig();

    /**
     * SwerveModule
     *
     * @param throttleID CAN ID of throttle
     * @param rotorID CAN ID of rotor
     * @param rotorEncoderID CAN ID of rotor encoder
     * @param rotorMagnetOffset rotor encoder
     */
    public SwerveModule(int throttleID, int rotorID, int rotorEncoderID, double rotorMagnetOffset)
    {
        // 實例化 throttle 馬達 & encoder
        mThrottle = new SparkMax(throttleID, MotorType.kBrushless);
        mThrottleEncoder = mThrottle.getEncoder();

        // 實例化 rotor 馬達
        mRotor = new SparkMax(rotorID, MotorType.kBrushless);

        // 實例化 rotor absolute encoder
        mRotorEncoder = new CANcoder(rotorEncoderID);

        drivingConfig.idleMode(IdleMode.kBrake).voltageCompensation(Constants.kVoltageCompensation);
        EncoderConfig drivingEncoderConfig = new EncoderConfig();
        drivingEncoderConfig.velocityConversionFactor(SwerveConstants.kThrottleVelocityConversionFactor);
        drivingEncoderConfig.velocityConversionFactor(SwerveConstants.kThrottlePositionConversionFactor);
        drivingConfig.encoder.apply(drivingEncoderConfig);
        turningConfig.idleMode(IdleMode.kBrake).voltageCompensation(Constants.kVoltageCompensation).inverted(SwerveConstants.kRotorMotorInversion);

        mRotor.configure(turningConfig, null, null);

        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        // magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        magnetSensorConfigs.SensorDirection = SwerveConstants.kRotorEncoderDirection;
        magnetSensorConfigs.MagnetOffset = rotorMagnetOffset;
        mRotorEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs), Constants.kLongTimeoutMs);

        mRotorPID = new PIDController(
            SwerveConstants.kRotor_kP,
            SwerveConstants.kRotor_kI,
            SwerveConstants.kRotor_kD
        );

        // ContinuousInput
        mRotorPID.enableContinuousInput(-180, 180);

        mThrottle.configure(drivingConfig, null, null);

        if (rotorEncoderID == 1) {
            m_name = "FrontLeft";
          }
          if (rotorEncoderID == 2) {
            m_name = "FrontRight";
          }
          if (rotorEncoderID == 3) {
            m_name = "BackRight";
            
          }
          if (rotorEncoderID == 4) {
            m_name = "BackLeft";
            
          }
    }

    /**
     * Return current state of module
     * 
     * @return module state
     */
    public SwerveModuleState getState() {
        SwerveModuleState state = new SwerveModuleState(
            mThrottleEncoder.getVelocity(),
            Rotation2d.fromRotations(mRotorEncoder.getAbsolutePosition().getValueAsDouble())
        );
        // SmartDashboard.putString(m_name + " current state: ", getState().toString());
        return state;
    }
    
    /**
     * Return current position of module
     * 
     * @return module position
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            mThrottleEncoder.getPosition(), 
            Rotation2d.fromRotations(mRotorEncoder.getAbsolutePosition().getValueAsDouble())
        );
    }

    /**
     * Set module state
     * 
     * @param state module state 
     */
    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = state;
        optimizedState.optimize(getState().angle);
    
        double rotorOutput = mRotorPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

        mRotor.set(rotorOutput);
        mThrottle.set(optimizedState.speedMetersPerSecond);
    }
}
