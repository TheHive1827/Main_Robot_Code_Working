package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.controller.ProfiledPIDController;
import java.io.File;
import java.util.Set;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;

import edu.wpi.first.math.util.Units;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.*;
import com.ctre.phoenix6.hardware.CANcoder;
// import swervelib.encoders.CANCoderSwerve;


public class ArmSubsystem { // Initializing physical swerve modules
  private final SparkMax m_ArmMotor;
  private double m_ArmEncoderPosition;
  private PIDController m_controller;
  int Position;
  int SetPoint;
  

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public ArmSubsystem(){
    m_ArmMotor = new SparkMax(ArmConstants.armID, MotorType.kBrushless);
    SmartDashboard.putNumber("Wowzers", m_ArmEncoderPosition);
    m_ArmEncoderPosition = m_ArmMotor.getEncoder().getPosition();
    int Position;
    private ProfiledPIDController final pidController = new ProfiledPIDController(SwerveConstants.kRotor_kP,
            SwerveConstants.kRotor_kI,
            SwerveConstants.kRotor_kD, ArmConstants.MOVEMENT_CONSTRAINTS);
            
    m_controller = new PIDController(SwerveConstants.kRotor_kP,
            SwerveConstants.kRotor_kI,
            SwerveConstants.kRotor_kD);
   m_controller.setReference(setPoint, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  public Command ArmSetCommand(double Position) {
    return runOnce(() ->m_controller.setReference(Position, SparkBase.ControlType.kMAXMotionPositionControl););
    }

  public void start(){
    Position = 1;
  }
}