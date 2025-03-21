// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.Constants.OIConstants;

// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.sim.SparkAbsoluteEncoderSim;
// import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
// import com.revrobotics.spark.SparkAbsoluteEncoder;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLowLevel;
// import com.revrobotics.spark.SparkMax;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkMaxAlternateEncoder;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.ctre.phoenix.motorcontrol.IFollower;
// import com.revrobotics.*;
// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.revrobotics.spark.SparkLowLevel.MotorType;


// public class ElevatorSubsystem extends SubsystemBase {
//   private final ArmSubsystem m_Arm = new ArmSubsystem();
//   static CommandXboxController ElevatorController = new CommandXboxController(1);
//   XboxController exampleXbox = new XboxController(1); // 0 is the USB Port to be used as indicated on the Driver Station
//   public final static SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ElevatorLeader, MotorType.kBrushless);
//   private final static RelativeEncoder elevatorMotorEncoder = elevatorMotor.getEncoder();

//   // pid stuff


//   double ElevatorEncoderValue = elevatorMotorEncoder.getPosition();
//   boolean ButtonA = exampleXbox.getAButtonPressed();
//   boolean ButtonY = exampleXbox.getYButtonPressed();
//   boolean ButtonX = exampleXbox.getXButtonPressed();
//   boolean ButtonB = exampleXbox.getBButtonPressed();

//   @Override
//   public void periodic() {
//     ButtonA = exampleXbox.getAButtonPressed();
//     ButtonY = exampleXbox.getYButtonPressed();
//     SmartDashboard.putNumber("Elevator encoder", elevatorMotorEncoder.getPosition());
//     SmartDashboard.putBoolean("A", ButtonA);
//     SmartDashboard.putBoolean("Y", ButtonY);
//   }

//   public Command ElevatorCommand(double speed) {
//     return runOnce(() -> elevatorMotor.set(speed));
//     }

//   // public void elevatorGetCoral() {
//   //   ElevatorController.b().whileTrue(
//   //     run(() -> ElevatorGoToPoint(ElevatorConstants.ElevatorGetCoralUp))
//   //   .andThen(run(() -> Commands.parallel(ElevatorGoToPoint(ElevatorConstants.ElevatorGetCoralDown),m_Arm.SetPosition(ArmConstants.armBottomGetCoral))))
//   //   .andThen(run(() -> m_Arm.SetPosition(ArmConstants.armMid)))
//   //   .andThen(run(() -> ElevatorGoToPoint(ElevatorConstants.ElevatorGetCoralUp))));}
// }
