package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.IFollower;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystemPos extends SubsystemBase {
  private final ArmSubsystem m_Arm = new ArmSubsystem();
  static CommandXboxController ElevatorController = new CommandXboxController(1);
  public final static SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ElevatorLeader, MotorType.kBrushless);
  private final static RelativeEncoder elevatorMotorEncoder = elevatorMotor.getEncoder();
  public static final SparkMaxConfig config = new SparkMaxConfig();

  public static SparkClosedLoopController m_elevatorPID = elevatorMotor.getClosedLoopController();
  RelativeEncoder encoder;
  public static final SparkMaxConfig motorConfig = new SparkMaxConfig();
  private double m_goal = 0.0;

  public void config() {
    m_elevatorPID = elevatorMotor.getClosedLoopController();

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(1.0)
        // speed
        .i(0.0)
        // integral
        .d(0.0)
        // kinda like friction
        .outputRange(-0.5, 0.5);

    elevatorMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator encoder", elevatorMotorEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Output", elevatorMotor.getAppliedOutput());
  }

  public void SetElevatorGoal(double position) {
    m_elevatorPID.setReference(position, ControlType.kPosition);
    m_goal = position;
  }

  public Command ElevatorCommand(double position) {
   return runOnce(() -> SetElevatorGoal(position));
  }

  public boolean ElevatorCloseToGoal() {
    double tolerance = 1.0;
    return Math.abs(elevatorMotorEncoder.getPosition() - m_goal) < tolerance;
  }

  public Command GetCoral() {
    return runOnce(() -> Commands.waitSeconds(0.01))
    .andThen(ElevatorCommand(ElevatorConstants.ElevatorGetCoralUp))
    .andThen(Commands.waitUntil(this::ElevatorCloseToGoal))
    .andThen(m_Arm.setPosition(ArmConstants.armMid))
    .andThen(Commands.waitUntil(this::ElevatorCloseToGoal))
        // .andThen(Commands.waitSeconds(2.0))
    // .andThen(Commands.waitUntil(this::ElevatorCloseToGoal));
    .andThen(m_Arm.setPosition(ArmConstants.armBottomGetCoral))
    .andThen(Commands.waitUntil(this::ElevatorCloseToGoal))
    .andThen(ElevatorCommand(ElevatorConstants.ElevatorGetCoralDown))
    .andThen(Commands.waitUntil(this::ElevatorCloseToGoal))
    .andThen(ElevatorCommand(ElevatorConstants.ElevatorGetCoralUp))
    .andThen(Commands.waitUntil(this::ElevatorCloseToGoal))
    .andThen(m_Arm.setPosition(ArmConstants.armMid))
    .andThen(Commands.waitUntil(this::ElevatorCloseToGoal));
  }

  public Command loadingPosition(){
    return run(() -> Commands.parallel(ElevatorCommand(ElevatorConstants.ElevatorMin), m_Arm.setPosition(ArmConstants.armUp)));
  }

  public void configureElevatorBindings() {
    ElevatorController.a().onTrue(ElevatorCommand(ElevatorConstants.ElevatorMin));
    ElevatorController.y().onTrue(ElevatorCommand(ElevatorConstants.ElevatorGetCoralUp));
    ElevatorController.x().onTrue(ElevatorCommand(ElevatorConstants.ElevatorScoreCoral));
    ElevatorController.leftBumper().onTrue(GetCoral()); 
    ElevatorController.b().onTrue(ElevatorCommand(ElevatorConstants.ElevatorGetCoralDown));
  }
}