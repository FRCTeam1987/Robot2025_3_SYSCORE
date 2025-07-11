// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.RobotContainer.ARM;
import static frc.robot.RobotContainer.DRIVETRAIN;

// import au.grapplerobotics.CanBridge;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
// import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SystemServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.autos.AutoHelpers;

public class Robot extends TimedRobot {
  private WrapperCommand AUTONOMOUS_COMMAND;

  private final RobotContainer ROBOT_CONTAINER;

  private double timeToCoast;

  public static boolean hasClimberCoasted = false;

  public static boolean hasRunAuto = false;

  public Robot() {
        // DogLog.setOptions(
        // new DogLogOptions().withLogExtras(false).withCaptureDs(false).withCaptureConsole(false));
    // CanBridge.runTCP();
    //RobotController.setBrownoutVoltage(6.0);
    SystemServer.getSystemServer().startEntryDataLog(DataLogManager.getLog(), "/Netcomm/", "NTSS:");
    FollowPathCommand.warmupCommand().schedule();
    PathfindingCommand.warmupCommand().schedule();
    ROBOT_CONTAINER = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    RobotContainer.updateLocalizationState();
    //DriverStation.reportWarning(String.valueOf(RobotContainer.JOYSTICK.getHID().getPOV()), false);
  }

  @Override
  public void disabledInit() {
    //    if (TEST_MODE) {
    //      LimelightHelpers.SetThrottle("limelight-scoring", 200);
    //    }
    DriverStation.reportWarning("DISABLED!", false);
    timeToCoast = Timer.getFPGATimestamp();
  }

  @Override
  public void disabledPeriodic() {
    if (timeToCoast + 15 < Timer.getFPGATimestamp() && !hasClimberCoasted) {
      RobotContainer.CLIMBER.coast();
      hasClimberCoasted = true;
    }
    if (timeToCoast + 5 < Timer.getFPGATimestamp()) {
      if (ARM.hasAlgae() || (ARM.hasGamePieceEntrance() && !ARM.hasGamePieceBack())) {
        ARM.coast();
      } else {
        ARM.brake();
      }
    } else {
      ARM.brake();
    }
  }

  @Override
  public void disabledExit() {
    RobotContainer.CLIMBER.brake();
    RobotContainer.ARM.brake();
    hasClimberCoasted = false;
  }

  @Override
  public void autonomousInit() {
    RobotContainer.autoTime = Timer.getFPGATimestamp();

    AUTONOMOUS_COMMAND = RobotContainer.getAutonomousCommand();

    AutoHelpers.matchTimeIncrement = Timer.getFPGATimestamp();

    if (AUTONOMOUS_COMMAND != null && hasRunAuto == false) {
      hasRunAuto = true;
      DRIVETRAIN.tareEverything();
      RobotContainer.VISION.setShouldUpdatePose(false);
      AUTONOMOUS_COMMAND.schedule();
    }
    if (hasRunAuto) {
      RobotContainer.VISION.setShouldUpdatePose(true);
    }
    RobotContainer.ELEVATOR.setConfigAuto();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    RobotContainer.autoTime = Timer.getFPGATimestamp();
    RobotContainer.ELEVATOR.setConfigTeleop();
    RobotContainer.VISION.setShouldUpdatePose(true);
    if (AUTONOMOUS_COMMAND != null) {
      AUTONOMOUS_COMMAND.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
