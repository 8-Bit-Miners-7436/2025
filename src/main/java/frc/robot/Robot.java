package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.CustomSwerveModule;

public class Robot extends TimedRobot {
  // * Initializes Xbox Controller and IMU (Gyro)
  private final XboxController gamepad = new XboxController(0);
  private final Pigeon2 pigeon = new Pigeon2(10);
  
  // * Initializes Kinematics Object w/ Swerve Module Offsets From Robot Center (In Meters)
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(0.34, 0.3), // front-left
    new Translation2d(0.34, -0.3),  // front-right
    new Translation2d(-0.34, 0.3),  // rear-left
    new Translation2d(-0.34, -0.3)    // rear-right
  );
    
  // * Initializes Swerve Modules And Package In Array For Easy Modular Access
  private final CustomSwerveModule[] modules = new CustomSwerveModule[] {
    new CustomSwerveModule(1), // front-left
    new CustomSwerveModule(2), // front-right
    new CustomSwerveModule(3), // rear-left
    new CustomSwerveModule(4)  // rear-right
  };

  // Todo: Make a More Permanent Elevator Solution
  private SparkMax elevatorMotor = new SparkMax(15, MotorType.kBrushless);

  @Override
  public void robotPeriodic() {
    double pigeonYaw = pigeon.getYaw(true).getValueAsDouble() % 360;
    double pigeonPitch = pigeon.getPitch(true).getValueAsDouble() % 360;
    double pigeonRoll = pigeon.getRoll(true).getValueAsDouble() % 360;
    SmartDashboard.putNumber("Pigeon Yaw", pigeonYaw);
    SmartDashboard.putNumber("Pigeon Pitch", pigeonPitch);
    SmartDashboard.putNumber("Pigeon Roll", pigeonRoll);
  }

  @Override
  public void teleopPeriodic() {
    double vx = gamepad.getLeftY(); // * Forward/backward
    double vy = gamepad.getLeftX(); // * Strafing
    double a = gamepad.getRightX(); // * Rotation

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, a);

    SwerveModuleState moduleStates[] = kinematics.toSwerveModuleStates(chassisSpeeds);

    for (int i = 0; i < modules.length; i++) {
      SwerveModuleState state = moduleStates[i];
      double targetAngle = state.angle.getRotations();
      double targetSpeed = state.speedMetersPerSecond;
      Double[] targetSpeedAndAngle = { targetAngle, targetSpeed };

      SmartDashboard.putNumberArray("Module #" + (i + 1), targetSpeedAndAngle);
      modules[i].setDriveSpeed(targetSpeed);
      modules[i].setTargetAngle(targetAngle);
    }

    // Todo: Make a More Permanent Elevator Solution
    if (gamepad.getAButton()) {
      elevatorMotor.set(1);
    } else if (gamepad.getBButton()) {
      elevatorMotor.set(-1);
    } else {
      elevatorMotor.stopMotor();
    }
  }

  @Override
  public void testInit() {
    // * Resets Swerve Module Steer Motor Encoders
    // * Start Test Mode Briefly After Re-Aligning Wheels Manually
    for (int i = 0; i < modules.length; i++) {
      modules[i].resetEncoder();
    }
  }
}
