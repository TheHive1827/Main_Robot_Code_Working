package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
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
  XboxController exampleXbox = new XboxController(1); // 0 is the USB Port to be used as indicated on the Driver Station
  public final static SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ElevatorLeader, MotorType.kBrushless);
  private final static RelativeEncoder elevatorMotorEncoder = elevatorMotor.getEncoder();
  public static final SparkMaxConfig config = new SparkMaxConfig();

  public static SparkClosedLoopController m_elevatorPID = elevatorMotor.getClosedLoopController();
  RelativeEncoder encoder;
  public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

  double ElevatorEncoderValue = elevatorMotorEncoder.getPosition();
  boolean ButtonA = exampleXbox.getAButtonPressed();
  boolean ButtonY = exampleXbox.getYButtonPressed();
  boolean ButtonX = exampleXbox.getXButtonPressed();
  boolean ButtonB = exampleXbox.getBButtonPressed();

   public void config(){
      m_elevatorPID = elevatorMotor.getClosedLoopController();
    encoder = elevatorMotor.getEncoder();

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.01)
        // speed
        .i(0.0)
        // integral 
        .d(0.0)
        // kinda like friction
        .outputRange(-1.0, 1);

    elevatorMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    ButtonA = exampleXbox.getAButtonPressed();
    ButtonY = exampleXbox.getYButtonPressed();
    
    SmartDashboard.putNumber("Elevator encoder", elevatorMotorEncoder.getPosition());
    SmartDashboard.putBoolean("A", ButtonA);
    SmartDashboard.putBoolean("Y", ButtonY);
  }

  public Command ElevatorCommand(double position) {
    return runOnce(() -> m_elevatorPID.setReference(position, ControlType.kPosition));
    }
  public void elevatorGetCoral() {
    ElevatorController.b().whileTrue(
      run(() -> ElevatorCommand(ElevatorConstants.ElevatorGetCoralUp))
    .andThen(run(() -> Commands.parallel(ElevatorCommand(ElevatorConstants.ElevatorGetCoralDown),m_Arm.SetPosition(ArmConstants.armBottomGetCoral))))
    .andThen(run(() -> m_Arm.SetPosition(ArmConstants.armMid)))
    .andThen(run(() -> ElevatorCommand(ElevatorConstants.ElevatorGetCoralUp))));}

    public void configureBindings() {
      ElevatorController.a().whileTrue(ElevatorCommand(ElevatorConstants.ElevatorMin));
      ElevatorController.y().whileTrue(ElevatorCommand(ElevatorConstants.ElevatorMax));
      ElevatorController.x().whileTrue(ElevatorCommand(ElevatorConstants.ElevatorMiddle));
}
}