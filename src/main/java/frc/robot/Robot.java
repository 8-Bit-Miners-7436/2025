package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.LoggedRobot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.CustomSwerveModule;

public class Robot extends LoggedRobot {
  public Robot() {
    // * Initialize AdvantageKit (Logger)
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
  }

  // * Initialize Xbox Controller and IMU (Gyro)
  private final XboxController gamepad = new XboxController(0);
  private final Pigeon2 pigeon = new Pigeon2(10);

  // * Create Kinematics Object with Module Offsets from Robot Center (in meters)
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(0.34, 0.3), // Front Left
      new Translation2d(0.34, -0.3), // Front Right
      new Translation2d(-0.34, 0.3), // Rear Left
      new Translation2d(-0.34, -0.3) // Rear Right
  );

  // * Initialize Swerve Modules and packages them in an Array for Easy Access
  private final CustomSwerveModule[] modules = new CustomSwerveModule[] {
      new CustomSwerveModule(1, "Front Left"),
      new CustomSwerveModule(2, "Front Right"),
      new CustomSwerveModule(3, "Rear Left"),
      new CustomSwerveModule(4, "Rear Right")
  };

  // Todo: Implement a more permanent elevator solution
  private final SparkMax elevatorMotor = new SparkMax(15, MotorType.kBrushless);
  private final SparkMax coralMotor = new SparkMax(16, MotorType.kBrushless);

  @Override
  public void robotPeriodic() {
    // * Logs Robot's Rotation/Facing Angle in AdvantageScope
    Logger.recordOutput("RobotRotation", pigeon.getRotation2d());
  }

  @Override
  public void teleopPeriodic() {
    // * Get Joystick Values for driving
    double vx = -gamepad.getLeftY(); // Forward/backward movement
    double vy = gamepad.getLeftX(); // Strafing movement
    double a = gamepad.getRightX(); // Rotational movement

    // * Convert Joystick Inputs to Chassis Speeds
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, a);

    // * Convert Chassis Speeds to individual Swerve Module States
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    // * logs Module States in AdvantageScope
    Logger.recordOutput("ModuleStates", moduleStates);

    // * Passes Swerve Module States to each Swerve Module
    for (int i = 0; i < modules.length; i++) {
      modules[i].updateState(moduleStates[i]);
    }

    // Todo: Implement a more permanent elevator solution
    if (gamepad.getAButton()) {
      elevatorMotor.set(0.25);
    } else if (gamepad.getBButton()) {
      elevatorMotor.set(-0.25);
    } else if (gamepad.getLeftTriggerAxis() > 0.01) {
      coralMotor.set(gamepad.getLeftTriggerAxis() / 5);
    } else if (gamepad.getRightTriggerAxis() > 0.01) {
      coralMotor.set(-gamepad.getRightTriggerAxis() / 4);
    } else {
      elevatorMotor.stopMotor();
      coralMotor.stopMotor();
    }
  }

  @Override
  public void testInit() {
    // * Resets All Swerve Module Steer-Motor Encoders
    // * Enable Test Mode briefly after manually re-aligning wheels
    for (CustomSwerveModule module : modules) {
      module.resetEncoder();
    }
  }
}