package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.lib.subsystem.SubsystemFault;
import frc.lib.swerve.PoseEstimator;
import frc.lib.swerve.SwerveSetpoint;
import frc.lib.util.Vector3;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve_module.SwerveModule;
import java.util.*;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.littletonrobotics.junction.Logger;

public class Swerve extends AdvancedSubsystem {
  private final PoseEstimator poseEstimator;
  private final SwerveModule[] modules;
  private final SwerveIO io;
  private final IMUOdomInputsAutoLogged imuOdomInputs = new IMUOdomInputsAutoLogged();
  private final IMUMiscInputsAutoLogged imuMiscInputs = new IMUMiscInputsAutoLogged();
  private final BaseStatusSignal[] allOdomSignals;
  private final OdometryThread odomThread;

  private final SwerveModulePosition[] currentPositions;
  private final SwerveModuleState[] currentStates;
  private ChassisSpeeds currentSpeeds;
  private final ReadWriteLock odomLock = new ReentrantReadWriteLock();

  private final boolean isSim = RobotBase.isSimulation();
  private final SysIdRoutine driveSysIDRoutine;
  private final SysIdRoutine rotationSysIDRoutine;

  private SwerveSetpoint prevSetpoint;

  public Swerve(SwerveIO io, SwerveModule... mods) {
    this.io = io;
    this.modules = mods;

    currentPositions = new SwerveModulePosition[modules.length];
    currentStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentPositions[i] = modules[i].getPosition();
      currentStates[i] = modules[i].getState();
    }
    currentSpeeds = Constants.Swerve.kinematics.toChassisSpeeds(currentStates);

    poseEstimator =
        new PoseEstimator(Constants.Swerve.PoseEstimator.stateStdDevs, Constants.Swerve.kinematics);

    // 4 signals for each module + 6 for Pigeon2
    if (RobotBase.isReal()) {
      allOdomSignals = new BaseStatusSignal[(4 * 4) + 6];
      for (int i = 0; i < modules.length; i++) {
        var signals = modules[i].getOdomSignals();
        allOdomSignals[(i * 4)] = signals[0];
        allOdomSignals[(i * 4) + 1] = signals[1];
        allOdomSignals[(i * 4) + 2] = signals[2];
        allOdomSignals[(i * 4) + 3] = signals[3];
      }
      var imuSignals = io.getOdomSignals();
      allOdomSignals[allOdomSignals.length - 6] = imuSignals[0];
      allOdomSignals[allOdomSignals.length - 5] = imuSignals[1];
      allOdomSignals[allOdomSignals.length - 4] = imuSignals[2];
      allOdomSignals[allOdomSignals.length - 3] = imuSignals[3];
      allOdomSignals[allOdomSignals.length - 2] = imuSignals[4];
      allOdomSignals[allOdomSignals.length - 1] = imuSignals[5];
    } else {
      allOdomSignals = new BaseStatusSignal[0];
    }

    for (var signal : allOdomSignals) {
      signal.setUpdateFrequency(Constants.Swerve.odomUpdateHz);
    }

    odomThread = new OdometryThread();

    if (!Logger.hasReplaySource()) {
      odomThread.start();
    }

    io.registerSelfCheckHardware(this);

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdom,
        this::getCurrentSpeeds,
        (speeds) -> driveRobotRelative(speeds, false),
        Constants.Swerve.pathFollowingConfig,
        Robot::isRedAlliance,
        this);

    driveSysIDRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4.0),
                null,
                state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {
                  for (SwerveModule module : modules) {
                    module.brakeRotation();
                    module.setDriveVoltage(volts.in(Volts));
                  }
                },
                null,
                this));

    rotationSysIDRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4.0),
                null,
                state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {
                  for (SwerveModule module : modules) {
                    module.brakeDrive();
                    module.setRotationVoltage(volts.in(Volts));
                  }
                },
                null,
                this));

    prevSetpoint = new SwerveSetpoint(currentSpeeds, currentStates);
  }

  @Override
  public void periodic() {
    long startTime = Logger.getRealTimestamp();

    Logger.recordOutput(
        "Swerve/ActiveCommand",
        getCurrentCommand() != null ? getCurrentCommand().getName() : "None");

    ChassisSpeeds currentSpeeds = getCurrentSpeeds();
    Logger.recordOutput(
        "Swerve/LinearVel",
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond));

    Pose2d pose = getPose();
    Rotation3d orientation = getOrientation();
    Logger.recordOutput("Swerve/Odom", pose);
    Logger.recordOutput("Swerve/UncorrectedOdom", poseEstimator.getOdomPose());
    Logger.recordOutput(
        "Swerve/Odom3d",
        new Pose3d(
            new Translation3d(pose.getX(), pose.getY(), 0.0),
            new Rotation3d(
                orientation.getX(), orientation.getY(), pose.getRotation().getRadians())));
    logStates("Swerve/CurrentStates", currentStates);
    logStates("Swerve/TargetStates", getTargetStates());

    if (DriverStation.isDisabled()) {
      prevSetpoint = new SwerveSetpoint(currentSpeeds, currentStates);
    }

    double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
    Logger.recordOutput("Swerve/PeriodicRuntimeMS", runtimeMS);
  }

  @Override
  public void simulationPeriodic() {
    if (Logger.hasReplaySource()) {
      // Update odom + inputs if we have a replay source since we don't run the odom thread in
      // replay
      Logger.processInputs("Swerve/IMU/Odom", imuOdomInputs);
      for (var module : modules) {
        module.updateOdometryInputs();
      }

      updateOdom();
    }
  }

  public Optional<Pose2d> getPoseAtTimestamp(double time) {
    odomLock.readLock().lock();
    Optional<Pose2d> pose = poseEstimator.sampleAt(time);
    odomLock.readLock().unlock();

    return pose;
  }

  public void logStates(String key, SwerveModuleState[] states) {
    double[] logged = new double[states.length * 2];
    int idx = 0;
    for (var state : states) {
      logged[idx] = state.angle.getRadians();
      logged[idx + 1] = state.speedMetersPerSecond;
      idx += 2;
    }

    Logger.recordOutput(key, logged);
  }

  public void resetOdom(Pose2d pose) {
    odomLock.writeLock().lock();
    poseEstimator.resetPose(pose);
    odomLock.writeLock().unlock();
  }

  public Rotation3d getOrientation() {
    return new Rotation3d(
        Units.degreesToRadians(imuOdomInputs.rollDeg),
        Units.degreesToRadians(imuOdomInputs.pitchDeg),
        Units.degreesToRadians(imuOdomInputs.yawDeg));
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(imuOdomInputs.yawDeg);
  }

  public Pose2d getPose() {
    odomLock.readLock().lock();
    Pose2d pose = poseEstimator.getEstimatedPose();
    odomLock.readLock().unlock();
    return pose;
  }

  public ChassisSpeeds getCurrentSpeeds() {
    odomLock.readLock().lock();
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            currentSpeeds.vxMetersPerSecond,
            currentSpeeds.vyMetersPerSecond,
            currentSpeeds.omegaRadiansPerSecond);
    odomLock.readLock().unlock();

    return speeds;
  }

  public double getLinearVelocity() {
    ChassisSpeeds speeds = getCurrentSpeeds();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelative, boolean useSetpointGenerator) {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, getPose().getRotation());
    driveRobotRelative(speeds, useSetpointGenerator);
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelative) {
    driveFieldRelative(fieldRelative, true);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelative, boolean useSetpointGenerator) {
    ChassisSpeeds discretized = ChassisSpeeds.discretize(robotRelative, 0.02);
    if (useSetpointGenerator) {
      prevSetpoint =
          Constants.Swerve.setpointGenerator.generateSetpoint(
              Constants.SwerveModule.kinematicLimits, prevSetpoint, discretized, 0.02);
      setModuleStates(prevSetpoint.moduleStates);
    } else {
      prevSetpoint = new SwerveSetpoint(getCurrentSpeeds(), currentStates);
      setModuleStates(Constants.Swerve.kinematics.toSwerveModuleStates(discretized));
    }
  }

  public void driveRobotRelative(ChassisSpeeds robotRelative) {
    driveRobotRelative(robotRelative, true);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(states[i]);
    }
  }

  public SwerveModuleState[] getTargetStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getTargetState();
    }
    return states;
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public void lockModules() {
    for (SwerveModule module : modules) {
      module.lockModule();
    }
  }

  public Vector3 getGravityVector() {
    return new Vector3(
        imuMiscInputs.gravVectorX, imuMiscInputs.gravVectorY, imuMiscInputs.gravVectorZ);
  }

  public void doOdometryCorrection(List<PoseEstimator.VisionObservation> visionUpdates) {
    odomLock.writeLock().lock();
    visionUpdates.stream()
        .sorted(Comparator.comparingDouble(PoseEstimator.VisionObservation::timestamp))
        .forEach(poseEstimator::addVisionObservation);
    odomLock.writeLock().unlock();
  }

  public Command driveToIntakeRing() {
    PIDController rotationController =
        new PIDController(Constants.Swerve.teleAngleHoldFactor, 0.0, 0.0);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    PIDController speedController = new PIDController(0.07, 0.0, 0.0);

    return Commands.sequence(
            RobotContainer.driveWithGamepad()
                .until(RobotContainer.llNotes::hasTarget), // Wait until we see note
            Commands.either(
                run(() -> {
                      if (!RobotContainer.llNotes.hasTarget()
                          || RobotContainer.intake.getCurrentDraw() >= 30) {
                        driveFieldRelative(new ChassisSpeeds(), true);
                        return;
                      }

                      var pose =
                          RobotContainer.swerve.getPoseAtTimestamp(
                              RobotContainer.llNotes.getMeasurementTimestamp());

                      if (pose.isEmpty()) {
                        driveFieldRelative(new ChassisSpeeds(), true);
                        return;
                      }

                      Rotation2d angleToNoteField =
                          pose.get()
                              .getRotation()
                              .plus(Rotation2d.fromDegrees(180))
                              .plus(RobotContainer.llNotes.getAngleToTarget());

                      Pose2d currentPose = getPose();

                      double angVel =
                          rotationController.calculate(
                              currentPose
                                  .getRotation()
                                  .plus(Rotation2d.fromDegrees(180))
                                  .getRadians(),
                              angleToNoteField.getRadians());
                      driveFieldRelative(new ChassisSpeeds(0, 0, angVel), false);
                    })
                    .withTimeout(0.25), // Robot is rotating, rotate towards note for a bit
                Commands.none(),
                () ->
                    Math.abs(getCurrentSpeeds().omegaRadiansPerSecond)
                        >= Units.degreesToRadians(180)),
            run(() -> {
                  if (!RobotContainer.llNotes.hasTarget()
                      || RobotContainer.intake.getCurrentDraw() >= 30) {
                    driveFieldRelative(new ChassisSpeeds(), true);
                    return;
                  }

                  var pose =
                      RobotContainer.swerve.getPoseAtTimestamp(
                          RobotContainer.llNotes.getMeasurementTimestamp());

                  if (pose.isEmpty()) {
                    driveFieldRelative(new ChassisSpeeds(), true);
                    return;
                  }

                  Rotation2d angleToNoteField =
                      pose.get()
                          .getRotation()
                          .plus(Rotation2d.fromDegrees(180))
                          .plus(RobotContainer.llNotes.getAngleToTarget());
                  Rotation2d ty = RobotContainer.llNotes.getTY();

                  Pose2d currentPose = getPose();

                  double angVel =
                      rotationController.calculate(
                          currentPose.getRotation().plus(Rotation2d.fromDegrees(180)).getRadians(),
                          angleToNoteField.getRadians());
                  double speed =
                      speedController.calculate(ty.plus(Rotation2d.fromDegrees(30)).getDegrees());

                  Translation2d fieldSpeeds =
                      new Translation2d(speed, angleToNoteField.plus(Rotation2d.fromDegrees(180)));

                  driveFieldRelative(
                      new ChassisSpeeds(fieldSpeeds.getX(), fieldSpeeds.getY(), angVel), true);
                })
                .onlyWhile(RobotContainer.llNotes::hasTarget))
        .repeatedly();
  }

  public void brakeMotors() {
    for (SwerveModule module : modules) {
      module.brakeMotors();
    }
  }

  public Command driveSysIDQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(
            () -> {
              if (Robot.isReal()) {
                SignalLogger.setPath("/U/sysID/SwerveDrive");
              }
              SignalLogger.start();
            }),
        Commands.run(
                () -> {
                  for (SwerveModule module : modules) {
                    module.setTargetRotation(0);
                  }
                })
            .raceWith(Commands.waitSeconds(1.0).andThen(driveSysIDRoutine.quasistatic(direction))));
  }

  public Command driveSysIDDynamic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(
            () -> {
              if (Robot.isReal()) {
                SignalLogger.setPath("/U/sysID/SwerveDrive");
              }
              SignalLogger.start();
            }),
        Commands.run(
                () -> {
                  for (SwerveModule module : modules) {
                    module.setTargetRotation(0);
                  }
                })
            .raceWith(Commands.waitSeconds(1.0).andThen(driveSysIDRoutine.dynamic(direction))));
  }

  public Command rotationSysIDQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(
            () -> {
              if (Robot.isReal()) {
                SignalLogger.setPath("/U/sysID/SwerveRotation");
              }
              SignalLogger.start();
            }),
        rotationSysIDRoutine.quasistatic(direction));
  }

  public Command rotationSysIDDynamic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(
            () -> {
              if (Robot.isReal()) {
                SignalLogger.setPath("/U/sysID/SwerveRotation");
              }
              SignalLogger.start();
            }),
        rotationSysIDRoutine.dynamic(direction));
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
            runOnce(
                () -> {
                  modules[0].getSystemCheckCommand().schedule();
                  modules[1].getSystemCheckCommand().schedule();
                  modules[2].getSystemCheckCommand().schedule();
                  modules[3].getSystemCheckCommand().schedule();
                }),
            // Hack to run module system checks since modules[] does not exist when this method is
            // called
            Commands.waitUntil(
                () ->
                    modules[0].getCurrentCommand() == null
                        && modules[1].getCurrentCommand() == null
                        && modules[2].getCurrentCommand() == null
                        && modules[3].getCurrentCommand() == null),
            run(() -> driveFieldRelative(new ChassisSpeeds(0, 0, 0.5))).withTimeout(2.0),
            run(() -> driveFieldRelative(new ChassisSpeeds(0, 0, -0.5))).withTimeout(2.0))
        .until(
            () ->
                !getFaults().isEmpty()
                    || !modules[0].getFaults().isEmpty()
                    || !modules[1].getFaults().isEmpty()
                    || !modules[2].getFaults().isEmpty()
                    || !modules[3].getFaults().isEmpty())
        .andThen(runOnce(this::brakeMotors));
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    List<ParentDevice> orchestra = new ArrayList<>(modules.length * 2);

    for (var module : modules) {
      orchestra.addAll(module.getOrchestraDevices());
    }

    return orchestra;
  }

  @Override
  public SystemStatus getSystemStatus() {
    SystemStatus worstStatus = SystemStatus.OK;

    for (SubsystemFault f : this.getFaults()) {
      if (f.sticky || f.timestamp > Timer.getFPGATimestamp() - 10) {
        if (f.isWarning) {
          if (worstStatus != SystemStatus.ERROR) {
            worstStatus = SystemStatus.WARNING;
          }
        } else {
          worstStatus = SystemStatus.ERROR;
        }
      }
    }

    for (SwerveModule module : modules) {
      SystemStatus moduleStatus = module.getSystemStatus();
      if (moduleStatus == SystemStatus.ERROR) {
        worstStatus = SystemStatus.ERROR;
      } else if (moduleStatus == SystemStatus.WARNING && worstStatus == SystemStatus.OK) {
        worstStatus = SystemStatus.WARNING;
      }
    }

    return worstStatus;
  }

  private void updateOdom() {
    for (int i = 0; i < modules.length; i++) {
      currentPositions[i] = modules[i].getPosition();
      currentStates[i] = modules[i].getState();
    }
    currentSpeeds = Constants.Swerve.kinematics.toChassisSpeeds(currentStates);

    poseEstimator.addOdometryObservation(
        new PoseEstimator.OdometryObservation(
            new SwerveDriveWheelPositions(currentPositions),
            getRotation(),
            imuOdomInputs.measurementTimestamp));
  }

  class OdometryThread extends Thread {
    private final int START_THREAD_PRIORITY = 1;

    MedianFilter peakRemover = new MedianFilter(3);
    LinearFilter lowPass = LinearFilter.movingAverage(50);
    long lastTimeMicSec = 0;
    long currentTimeMicSec = Logger.getRealTimestamp();
    double averageLoopTimeSec = 0;

    int lastThreadPriority = START_THREAD_PRIORITY;
    int threadPriorityToSet = START_THREAD_PRIORITY;

    public OdometryThread() {
      super();
    }

    @Override
    public void run() {
      Threads.setCurrentThreadPriority(true, START_THREAD_PRIORITY);

      while (true) {
        updateAllOdomInputs();

        lastTimeMicSec = currentTimeMicSec;
        currentTimeMicSec = Logger.getRealTimestamp();
        double timeDeltaSec = (currentTimeMicSec - lastTimeMicSec) / 1000000.0;
        averageLoopTimeSec = lowPass.calculate(peakRemover.calculate(timeDeltaSec));

        odomLock.writeLock().lock();
        updateOdom();
        odomLock.writeLock().unlock();

        Logger.recordOutput("Swerve/OdomPeriod", averageLoopTimeSec);
        Logger.recordOutput("Swerve/OdomHz", Math.round(1.0 / averageLoopTimeSec));

        if (threadPriorityToSet != lastThreadPriority) {
          Threads.setCurrentThreadPriority(true, threadPriorityToSet);
          lastThreadPriority = threadPriorityToSet;
        }
      }
    }

    private void updateAllOdomInputs() {
      if (isSim) {
        try {
          // -1 here cuz the thread will tend to sleep 1ms longer than specified
          Thread.sleep((long) ((1.0 / Constants.Swerve.odomUpdateHz) * 1000.0) - 1);
        } catch (InterruptedException ignored) {
        }

        io.setCurrentAngularVel(Units.radiansToDegrees(getCurrentSpeeds().omegaRadiansPerSecond));
      } else {
        BaseStatusSignal.waitForAll(2.0 / Constants.Swerve.odomUpdateHz, allOdomSignals);
      }

      io.updateIMUOdomInputs(imuOdomInputs);
      Logger.processInputs("Swerve/IMU/Odom", imuOdomInputs);

      for (var module : modules) {
        module.updateOdometryInputs();
      }
    }

    public void setThreadPriority(int priority) {
      threadPriorityToSet = priority;
    }
  }
}
