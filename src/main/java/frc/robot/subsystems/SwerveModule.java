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
     // 初始化 rotor & throttle 馬達
    private SparkMax mRotor;
    private SparkMax mThrottle;

    // 初始化 throttle encoder
    private RelativeEncoder mThrottleEncoder;

    // 初始化 rotor encoder
    private CANcoder mRotorEncoder;

    // 初始化 rotor PID controller
    private PIDController mRotorPID;

    private String m_name;

    public SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public SparkMaxConfig turningConfig = new SparkMaxConfig();

    /**
     * 構建新的 SwerveModule
     *
     * @param throttleID CAN ID of throttle 馬達
     * @param rotorID CAN ID of rotor 馬達
     * @param rotorEncoderID CAN ID of rotor encoder
     * @param rotorMagnetOffset rotor encoder 偏移量
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

        // 根據之前的常數配置轉向 rotor encoder
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        // magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        magnetSensorConfigs.SensorDirection = SwerveConstants.kRotorEncoderDirection;
        magnetSensorConfigs.MagnetOffset = rotorMagnetOffset;
        mRotorEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs), Constants.kLongTimeoutMs);

        // 根據之前的常數配置 rotor 馬達的PID控制器
        mRotorPID = new PIDController(
            SwerveConstants.kRotor_kP,
            SwerveConstants.kRotor_kI,
            SwerveConstants.kRotor_kD
        );

        // ContinuousInput 認為 min 和 max 是同一點並且自動計算到設定點的最短路線
        mRotorPID.enableContinuousInput(-180, 180);

        // 根據之前的常數配置 throttle 馬達
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
        // 優化狀態，使轉向馬達不必旋轉超過 90 度來獲得目標的角度
        SmartDashboard.putNumber(m_name + " state angle: ", state.angle.getDegrees());
        SwerveModuleState optimizedState = state;
        optimizedState.optimize(getState().angle);
        
        // 通過比較目前角度與目標角度來用 PID 控制器計算轉向馬達所需的輸出
        SmartDashboard.putNumber(m_name + " current angle: ", getState().angle.getDegrees());
        SmartDashboard.putNumber(m_name + " optimized angle: ", optimizedState.angle.getDegrees());
        double rotorOutput = mRotorPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());
        SmartDashboard.putNumber(m_name + " Rotor output: ", rotorOutput);

        mRotor.set(rotorOutput);
        mThrottle.set(optimizedState.speedMetersPerSecond);

        // System.out.println("Desired State: " + optimizedState);
        SmartDashboard.putString(m_name + " desired state: ", optimizedState.toString());
        SmartDashboard.putString(m_name + " current state: ", getState().toString());
    }
}
