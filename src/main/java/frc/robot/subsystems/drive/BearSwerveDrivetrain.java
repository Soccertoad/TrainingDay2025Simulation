/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.robot.subsystems.drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.IntSupplier;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SimSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveRequest.NativeSwerveRequest;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.ctre.BearSwerveModuleConstants;
import frc.robot.util.ctre.SwerveDrivetrainConstants;

/**
 * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API.
 * <p>
 * This class handles the kinematics, configuration, and odometry of a
 * swerve drive utilizing CTR Electronics devices. We recommend
 * that users use the Swerve Mechanism Generator in Tuner X to create
 * a template project that demonstrates how to use this class.
 * <p>
 * This class will construct the hardware devices internally, so the user
 * only specifies the constants (IDs, PID gains, gear ratios, etc).
 * Getters for these hardware devices are available.
 * <p>
 * If using the generator, the order in which modules are constructed is
 * Front Left, Front Right, Back Left, Back Right. This means if you need
 * the Back Left module, call {@code getModule(2);} to get the 3rd index
 * (0-indexed) module, corresponding to the Back Left module.
 */
public class BearSwerveDrivetrain implements AutoCloseable {
    /**
     * Plain-Old-Data class holding the state of the swerve drivetrain.
     * This encapsulates most data that is relevant for telemetry or
     * decision-making from the Swerve Drive.
     */
    public static class SwerveDriveState {
        /** The current pose of the robot */
        public Pose2d Pose = new Pose2d();
        /** The current velocity of the robot */
        public ChassisSpeeds Speeds = new ChassisSpeeds();
        /** The current module states */
        public SwerveModuleState[] ModuleStates;
        /** The target module states */
        public SwerveModuleState[] ModuleTargets;
        /** The current module positions */
        public SwerveModulePosition[] ModulePositions;
        /** The measured odometry update period, in seconds */
        public double OdometryPeriod;
        /** Number of successful data acquisitions */
        public int SuccessfulDaqs;
        /** Number of failed data acquisitions */
        public int FailedDaqs;

        private void updateFromJni(SwerveJNI.DriveState driveState) {
            if (Pose.getX() != driveState.PoseX ||
                Pose.getY() != driveState.PoseY ||
                Pose.getRotation().getRadians() != driveState.PoseTheta)
            {
                Pose = new Pose2d(driveState.PoseX, driveState.PoseY, Rotation2d.fromRadians(driveState.PoseTheta));
            }

            if (Speeds.vxMetersPerSecond != driveState.SpeedsVx ||
                Speeds.vyMetersPerSecond != driveState.SpeedsVy ||
                Speeds.omegaRadiansPerSecond != driveState.SpeedsOmega)
            {
                Speeds = new ChassisSpeeds(driveState.SpeedsVx, driveState.SpeedsVy, driveState.SpeedsOmega);
            }

            for (int i = 0; i < ModuleStates.length; ++i) {
                if (ModuleStates[i].speedMetersPerSecond != driveState.ModuleStates[i].speed ||
                    ModuleStates[i].angle.getRadians() != driveState.ModuleStates[i].angle)
                {
                    ModuleStates[i] = new SwerveModuleState(driveState.ModuleStates[i].speed, Rotation2d.fromRadians(driveState.ModuleStates[i].angle));
                }
                if (ModuleTargets[i].speedMetersPerSecond != driveState.ModuleTargets[i].speed ||
                    ModuleTargets[i].angle.getRadians() != driveState.ModuleTargets[i].angle)
                {
                    ModuleTargets[i] = new SwerveModuleState(driveState.ModuleTargets[i].speed, Rotation2d.fromRadians(driveState.ModuleTargets[i].angle));
                }
                if (ModulePositions[i].distanceMeters != driveState.ModulePositions[i].distance ||
                    ModulePositions[i].angle.getRadians() != driveState.ModulePositions[i].angle)
                {
                    ModulePositions[i] = new SwerveModulePosition(driveState.ModulePositions[i].distance, Rotation2d.fromRadians(driveState.ModulePositions[i].angle));
                }
            }

            OdometryPeriod = driveState.OdometryPeriod;
            SuccessfulDaqs = driveState.SuccessfulDaqs;
            FailedDaqs = driveState.FailedDaqs;
        }
    }

    /*
     * Contains everything the control requests need to calculate the module state.
     */
    public static class BearSwerveControlParameters extends SwerveControlParameters {
        public SwerveDriveKinematics kinematics;
        public Translation2d[] swervePositions;
        public double kMaxSpeedMps;

        public Rotation2d operatorForwardDirection = new Rotation2d();
        public ChassisSpeeds currentChassisSpeed = new ChassisSpeeds();
        public Pose2d currentPose = new Pose2d();
        public double timestamp;
        public double updatePeriod;

        public void updateFromJni(SwerveJNI.ControlParams controlParams) {
            kMaxSpeedMps = controlParams.kMaxSpeedMps;
            if (operatorForwardDirection.getRadians() != controlParams.operatorForwardDirection) {
                operatorForwardDirection = Rotation2d.fromRadians(controlParams.operatorForwardDirection);
            }
            currentChassisSpeed.vxMetersPerSecond = controlParams.currentChassisSpeedVx;
            currentChassisSpeed.vyMetersPerSecond = controlParams.currentChassisSpeedVy;
            currentChassisSpeed.omegaRadiansPerSecond = controlParams.currentChassisSpeedOmega;
            if (currentPose.getX() != controlParams.currentPoseX ||
                currentPose.getY() != controlParams.currentPoseY ||
                currentPose.getRotation().getRadians() != controlParams.currentPoseTheta)
            {
                currentPose = new Pose2d(
                    controlParams.currentPoseX,
                    controlParams.currentPoseY,
                    Rotation2d.fromRadians(controlParams.currentPoseTheta)
                );
            }
            timestamp = controlParams.timestamp;
            updatePeriod = controlParams.updatePeriod;
        }
    }

    protected final int m_drivetrain;
    protected final SwerveJNI m_jni = new SwerveJNI();

    protected final SwerveModule[] m_modules;
    protected final Translation2d[] m_moduleLocations;

    protected final SwerveDriveKinematics m_kinematics;

    protected final Pigeon2 m_pigeon2;
    protected final SimSwerveDrivetrain m_simDrive;

    protected final BearSwerveControlParameters m_controlParams = new BearSwerveControlParameters();
    protected SwerveRequest m_swerveRequest = new SwerveRequest.Idle();
    protected long m_controlHandle = 0;

    protected final SwerveJNI m_telemetryJNI;
    protected Consumer<SwerveDriveState> m_telemetryFunction = null;
    protected long m_telemetryHandle = 0;

    protected final Lock m_stateLock = new ReentrantLock();
    protected final SwerveDriveState m_cachedState = new SwerveDriveState();

    /** Performs swerve module updates in a separate thread to minimize latency. */
    public class OdometryThread {
        /**
         * Starts the odometry thread.
         */
        public void start() {
            SwerveJNI.JNI_Odom_Start(m_drivetrain);
        }

        /**
         * Stops the odometry thread.
         */
        public void stop() {
            SwerveJNI.JNI_Odom_Stop(m_drivetrain);
        }

        public boolean isOdometryValid() {
            return SwerveJNI.JNI_IsOdometryValid(m_drivetrain);
        }

        /**
         * Sets the DAQ thread priority to a real time priority under the specified priority level
         *
         * @param priority Priority level to set the DAQ thread to.
         *                 This is a value between 0 and 99, with 99 indicating higher priority and 0 indicating lower priority.
         */
        public void setThreadPriority(int priority) {
            SwerveJNI.JNI_Odom_SetThreadPriority(m_drivetrain, priority);
        }
    }

    protected final OdometryThread m_odometryThread = new OdometryThread();

    private Thread m_shutdownHook;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public BearSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... modules) {
        this(() -> {
                long nativeDriveConstants = drivetrainConstants.createNativeInstance();
                long nativeModuleConstants = BearSwerveModuleConstants.createNativeInstance(modules);

                var drivetrain = SwerveJNI.JNI_CreateDrivetrain(nativeDriveConstants, nativeModuleConstants, modules.length);

                SwerveJNI.JNI_DestroyConstants(nativeDriveConstants);
                SwerveJNI.JNI_DestroyConstants(nativeModuleConstants);
                return drivetrain;
            }, drivetrainConstants, modules
        );
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public BearSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        this(() -> {
                long nativeDriveConstants = drivetrainConstants.createNativeInstance();
                long nativeModuleConstants = BearSwerveModuleConstants.createNativeInstance(modules);

                var drivetrain = SwerveJNI.JNI_CreateDrivetrainWithFreq(nativeDriveConstants, odometryUpdateFrequency,
                        nativeModuleConstants, modules.length);

                SwerveJNI.JNI_DestroyConstants(nativeDriveConstants);
                SwerveJNI.JNI_DestroyConstants(nativeModuleConstants);
                return drivetrain;
            }, drivetrainConstants, modules
        );
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     * @param visionStandardDeviation    The standard deviation for vision calculation
     * @param modules                    Constants for each specific module
     */
    public BearSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants... modules) {
        this(() -> {
                long nativeDriveConstants = drivetrainConstants.createNativeInstance();
                long nativeModuleConstants = BearSwerveModuleConstants.createNativeInstance(modules);

                var drivetrain = SwerveJNI.JNI_CreateDrivetrainWithStddev(nativeDriveConstants, odometryUpdateFrequency,
                        odometryStandardDeviation.getData(), visionStandardDeviation.getData(),
                        nativeModuleConstants, modules.length);

                SwerveJNI.JNI_DestroyConstants(nativeDriveConstants);
                SwerveJNI.JNI_DestroyConstants(nativeModuleConstants);
                return drivetrain;
            }, drivetrainConstants, modules
        );
    }

    private BearSwerveDrivetrain(IntSupplier createNativeInst, SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... modules) {
        m_drivetrain = createNativeInst.getAsInt();

        m_cachedState.ModuleStates = new SwerveModuleState[modules.length];
        m_cachedState.ModuleTargets = new SwerveModuleState[modules.length];
        m_cachedState.ModulePositions = new SwerveModulePosition[modules.length];
        m_jni.driveState.ModuleStates = new SwerveJNI.ModuleState[modules.length];
        m_jni.driveState.ModuleTargets = new SwerveJNI.ModuleState[modules.length];
        m_jni.driveState.ModulePositions = new SwerveJNI.ModulePosition[modules.length];
        for (int i = 0; i < modules.length; ++i) {
            m_cachedState.ModuleStates[i] = new SwerveModuleState();
            m_cachedState.ModuleTargets[i] = new SwerveModuleState();
            m_cachedState.ModulePositions[i] = new SwerveModulePosition();
            m_jni.driveState.ModuleStates[i] = new SwerveJNI.ModuleState();
            m_jni.driveState.ModuleTargets[i] = new SwerveJNI.ModuleState();
            m_jni.driveState.ModulePositions[i] = new SwerveJNI.ModulePosition();
        }

        m_telemetryJNI = m_jni.clone();

        m_modules = new SwerveModule[modules.length];
        m_moduleLocations = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; ++i) {
            m_modules[i] = new SwerveModule(modules[i], drivetrainConstants.CANBusName, m_drivetrain, i);
            m_moduleLocations[i] = new Translation2d(modules[i].LocationX, modules[i].LocationY);
        }

        m_kinematics = new SwerveDriveKinematics(m_moduleLocations);

        m_controlParams.kinematics = m_kinematics;
        m_controlParams.swervePositions = m_moduleLocations;

        m_pigeon2 = new Pigeon2(drivetrainConstants.Pigeon2Id, drivetrainConstants.CANBusName);
        m_simDrive = new MapleSimSwerveDrivetrain(m_moduleLocations, m_pigeon2.getSimState(), modules);

        if (drivetrainConstants.Pigeon2Configs != null) {
            var retval = getPigeon2().getConfigurator().apply(drivetrainConstants.Pigeon2Configs);
            if (!retval.isOK()) {
                System.out.println("Pigeon2 ID " + getPigeon2().getDeviceID() + " failed config with error: " + retval);
            }
        }
        /* do not start thread until after applying Pigeon 2 configs */
        m_odometryThread.start();

        m_shutdownHook = new Thread(() -> {
            m_shutdownHook = null;
            close();
        });
        Runtime.getRuntime().addShutdownHook(m_shutdownHook);
    }

    @Override
    public void close() {
        SwerveJNI.JNI_DestroyDrivetrain(m_drivetrain);
        if (m_controlHandle != 0) {
            SwerveJNI.JNI_DestroyControl(m_controlHandle);
            m_controlHandle = 0;
        }
        if (m_telemetryHandle != 0) {
            SwerveJNI.JNI_DestroyTelemetry(m_telemetryHandle);
            m_telemetryHandle = 0;
        }

        if (m_shutdownHook != null) {
            Runtime.getRuntime().removeShutdownHook(m_shutdownHook);
            m_shutdownHook = null;
        }
    }

    /**
     * Updates all the simulation state variables for this
     * drivetrain class. User provides the update variables for the simulation.
     *
     * @param dtSeconds time since last update call
     * @param supplyVoltage voltage as seen at the motor controllers
     */
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        m_simDrive.update(dtSeconds, supplyVoltage, m_modules);
    }

    /**
     * Gets a reference to the data acquisition thread.
     *
     * @return DAQ thread
     */
    public OdometryThread getDaqThread() {
        return m_odometryThread;
    }

    /**
     * Applies the specified control request to this swerve drivetrain.
     *
     * @param request Request to apply
     */
    public void setControl(SwerveRequest request) {
        if (m_swerveRequest != request) {
            m_swerveRequest = request;

            var prevControlHandle = m_controlHandle;

            if (request == null) {
                m_controlHandle = m_jni.JNI_SetControl(m_drivetrain, null);
            } else if (request instanceof NativeSwerveRequest req) {
                req.applyNative(m_drivetrain);
                m_controlHandle = 0;
            } else {
                m_controlHandle = m_jni.JNI_SetControl(m_drivetrain, () -> {
                    m_controlParams.updateFromJni(m_jni.controlParams);
                    return request.apply(m_controlParams, m_modules).value;
                });
            }

            if (prevControlHandle != 0) {
                SwerveJNI.JNI_DestroyControl(prevControlHandle);
            }
        } else if (request instanceof NativeSwerveRequest req) {
            /* update the native object */
            req.applyNative(m_drivetrain);
        }
    }

    /**
     * Configures the neutral mode to use for all modules' drive motors.
     *
     * @param neutralMode The drive motor neutral mode
     * @return Status code of the first failed config call, or OK if all succeeded
     */
    public StatusCode configNeutralMode(NeutralModeValue neutralMode) {
        return StatusCode.valueOf(SwerveJNI.JNI_ConfigNeutralMode(m_drivetrain, neutralMode.value));
    }

    /**
     * Zero's this swerve drive's odometry entirely.
     * <p>
     * This will zero the entire odometry, and place the robot at 0,0
     */
    public void tareEverything() {
        SwerveJNI.JNI_TareEverything(m_drivetrain);
    }

    /**
     * Takes the current orientation of the robot and makes it X forward for
     * field-relative maneuvers.
     */
    public void seedFieldRelative() {
        SwerveJNI.JNI_SeedFieldRelative(m_drivetrain);
    }

    /**
     * Takes the specified location and makes it the current pose for
     * field-relative maneuvers
     *
     * @param location Pose to make the current pose
     */
    public void seedFieldRelative(Pose2d location) {
        SwerveJNI.JNI_SeedFieldRelativeTo(m_drivetrain, location.getX(), location.getY(), location.getRotation().getRadians());
    }

    /**
     * Takes the {@link SwerveRequest.ForwardReferenceValue#RedAlliance} perpective direction
     * and treats it as the forward direction for
     * {@link SwerveRequest.ForwardReferenceValue#OperatorPerspective}.
     * <p>
     * If the operator is in the Blue Alliance Station, this should be 0 degrees.
     * If the operator is in the Red Alliance Station, this should be 180 degrees.
     *
     * @param fieldDirection Heading indicating which direction is forward from
     *                       the {@link SwerveRequest.ForwardReferenceValue#RedAlliance} perspective
     */
    public void setOperatorPerspectiveForward(Rotation2d fieldDirection) {
        SwerveJNI.JNI_SetOperatorPerspectiveForward(m_drivetrain, fieldDirection.getRadians());
    }

    /**
     * Check if the odometry is currently valid
     *
     * @return True if odometry is valid
     */
    public boolean isOdometryValid() {
        return SwerveJNI.JNI_IsOdometryValid(m_drivetrain);
    }

    /**
     * Get a reference to the module at the specified index.
     * The index corresponds to the module described in the constructor.
     *
     * @param index Which module to get
     * @return Reference to SwerveModule
     */
    public SwerveModule getModule(int index) {
        if (index >= m_modules.length) {
            return null;
        }
        return m_modules[index];
    }

    /**
     * Get a reference to the full array of modules.
     * The indexes correspond to the module described in the constructor.
     *
     * @return Reference to the SwerveModule array
     */
    public SwerveModule[] getModules() {
        return m_modules;
    }

    /**
     * Gets the current state of the swerve drivetrain.
     *
     * @return Current state of the drivetrain
     */
    public SwerveDriveState getState() {
        m_jni.JNI_GetState(m_drivetrain);
        try {
            m_stateLock.lock();
            m_cachedState.updateFromJni(m_jni.driveState);
            return m_cachedState;
        } finally {
            m_stateLock.unlock();
        }
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     * <p>
     * This method can be called as infrequently as you want, as long as you are
     * calling {@link SwerveDrivePoseEstimator#update} every loop.
     * <p>
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we recommend only adding vision measurements that are already within
     * one meter or so of the current pose estimate.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds. Note that if you don't use your own
     *                              time source by calling {@link
     *                              SwerveDrivePoseEstimator#updateWithTime(double,Rotation2d,SwerveModulePosition[])}
     *                              then you must use a timestamp with an epoch
     *                              since system startup (i.e., the epoch of this
     *                              timestamp is the same epoch as
     *                              {@link Utils#getCurrentTimeSeconds}.)
     *                              This means that you should use
     *                              {@link Utils#getCurrentTimeSeconds}
     *                              as your time source or sync the epochs.
     *                              An FPGA timestamp can be converted to the correct
     *                              timebase using {@link Utils#fpgaToCurrentTime}.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        SwerveJNI.JNI_AddVisionMeasurement(m_drivetrain, visionRobotPoseMeters.getX(), visionRobotPoseMeters.getY(),
            visionRobotPoseMeters.getRotation().getRadians(), timestampSeconds);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     * <p>
     * This method can be called as infrequently as you want, as long as you are
     * calling {@link SwerveDrivePoseEstimator#update} every loop.
     * <p>
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we recommend only adding vision measurements that are already within
     * one meter or so of the current pose estimate.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to {@link
     * SwerveDrivePoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds. Note that if you don't use your own
     *                                 time source by calling {@link
     *                                 SwerveDrivePoseEstimator#updateWithTime(double,Rotation2d,SwerveModulePosition[])}
     *                                 then you must use a timestamp with an epoch
     *                                 since system startup (i.e., the epoch of this
     *                                 timestamp is the same epoch as
     *                                 {@link Utils#getCurrentTimeSeconds}.)
     *                                 This means that you should use
     *                                 {@link Utils#getCurrentTimeSeconds}
     *                                 as your time source or sync the epochs.
     *                                 An FPGA timestamp can be converted to the correct
     *                                 timebase using {@link Utils#fpgaToCurrentTime}.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement (x position
     *                                 in meters, y position in meters, and heading
     *                                 in radians). Increase these numbers to trust
     *                                 the vision pose measurement less.
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs)
    {
        SwerveJNI.JNI_AddVisionMeasurementWithStdDev(m_drivetrain, visionRobotPoseMeters.getX(), visionRobotPoseMeters.getY(),
            visionRobotPoseMeters.getRotation().getRadians(), timestampSeconds, visionMeasurementStdDevs.getData());
    }

    /**
     * Sets the pose estimator's trust of global measurements. This might be used to
     * change trust in vision measurements after the autonomous period, or to change
     * trust as distance to a vision target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision
     *                                 measurements. Increase these numbers to
     *                                 trust global measurements from vision less.
     *                                 This matrix is in the form [x, y, theta]ᵀ,
     *                                 with units in meters and radians.
     */
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        SwerveJNI.JNI_SetVisionMeasurementStdDevs(m_drivetrain, visionMeasurementStdDevs.getData());
    }

    /**
     * Register the specified lambda to be executed whenever our SwerveDriveState function
     * is updated in our odometry thread.
     * <p>
     * It is imperative that this function is cheap, as it will be executed along with
     * the odometry call, and if this takes a long time, it may negatively impact
     * the odometry of this stack.
     * <p>
     * This can also be used for logging data if the function performs logging instead of telemetry
     *
     * @param telemetryFunction Function to call for telemetry or logging
     */
    public void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
        if (m_telemetryFunction != telemetryFunction) {
            m_telemetryFunction = telemetryFunction;

            var prevTelemetryHandle = m_telemetryHandle;

            if (telemetryFunction == null) {
                m_telemetryHandle = m_telemetryJNI.JNI_RegisterTelemetry(m_drivetrain, null);
            } else {
                m_telemetryHandle = m_telemetryJNI.JNI_RegisterTelemetry(m_drivetrain, () -> {
                    try {
                        m_stateLock.lock();
    
                        m_cachedState.updateFromJni(m_telemetryJNI.driveState);
                        telemetryFunction.accept(m_cachedState);
                    } finally {
                        m_stateLock.unlock();
                    }
                });
            }

            if (prevTelemetryHandle != 0) {
                SwerveJNI.JNI_DestroyTelemetry(prevTelemetryHandle);
            }
        }
    }

    /**
     * Gets the current orientation of the robot as a {@link Rotation3d} from
     * the Pigeon 2 quaternion values.
     *
     * @return The robot orientation as a {@link Rotation3d}
     */
    public Rotation3d getRotation3d() {
        return m_pigeon2.getRotation3d();
    }

    /**
     * Gets this drivetrain's Pigeon 2 reference.
     * <p>
     * This should be used only to access signals and change configurations that the
     * swerve drivetrain does not configure itself.
     *
     * @return This drivetrain's Pigeon 2 reference
     */
    public Pigeon2 getPigeon2() {
        return m_pigeon2;
    }
}
