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
  private XboxController gamepad;
  private Pigeon2 pigeon;
  private Translation2d frontLeftLocation;
  private Translation2d frontRightLocation;
  private Translation2d rearLeftLocation;
  private Translation2d rearRightLocation;

  private SwerveDriveKinematics kinematics;
  private SwerveModuleState[] moduleStates;
  private CustomSwerveModule[] modules;

  private double pigeonYaw;
  private double pigeonPitch;
  private double pigeonRoll;

  private SparkMax elevatorMotor;

  @Override
  public void robotInit() {
    gamepad = new XboxController(0);
    pigeon = new Pigeon2(10);
    frontLeftLocation = new Translation2d(0.34, 0.3);
    frontRightLocation = new Translation2d(0.34, -0.3);
    rearLeftLocation = new Translation2d(-0.34, 0.3);
    rearRightLocation = new Translation2d(-0.34, -0.3);

    kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, rearLeftLocation, rearRightLocation);

    modules = new CustomSwerveModule[] {
        new CustomSwerveModule(1), // front-left
        new CustomSwerveModule(2), // front-right
        new CustomSwerveModule(3), // rear-left
        new CustomSwerveModule(4), // rear-right
    };

    elevatorMotor = new SparkMax(15, MotorType.kBrushless);
  }

  @Override
  public void robotPeriodic() {
    pigeonYaw = pigeon.getYaw().getValueAsDouble() % 360;
    pigeonPitch = pigeon.getPitch().getValueAsDouble() % 360;
    pigeonRoll = pigeon.getRoll().getValueAsDouble() % 360;
    SmartDashboard.putNumber("Pigeon Yaw", pigeonYaw);
    SmartDashboard.putNumber("Pigeon Pitch", pigeonPitch);
    SmartDashboard.putNumber("Pigeon Roll", pigeonRoll);
  }

  @Override
  public void teleopPeriodic() {
    double vx = gamepad.getLeftY(); // * Forward/backward
    double vy = gamepad.getLeftX(); // * Strafing
    double a = gamepad.getRightX(); // / Math.abs(pigeonYaw/360); // * Rotation

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, a);

    moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    for (int i = 0; i < modules.length; i++) {
      SwerveModuleState state = moduleStates[i];
      double targetAngle = state.angle.getRotations();
      double targetSpeed = state.speedMetersPerSecond;
      Double[] targetSpeedAndAngle = { targetAngle, targetSpeed };

      SmartDashboard.putNumberArray("Module #" + (i + 1), targetSpeedAndAngle);
      modules[i].setDriveSpeed(targetSpeed);
      modules[i].setTargetAngle(targetAngle);
    }
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
    for (int i = 0; i < modules.length; i++) {
      modules[i].resetEncoder();
    }
  }
}
