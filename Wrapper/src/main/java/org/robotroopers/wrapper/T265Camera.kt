package org.robotroopers.wrapper

import android.content.Context
import android.util.Log
import com.asiankoala.koawalib.math.Pose
import com.intel.realsense.librealsense.DeviceListener
import com.intel.realsense.librealsense.RsContext
import java.util.concurrent.atomic.AtomicBoolean
import java.util.function.Consumer
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

/**
 * Provides a convenient Java interface to the Intel RealSense T265 V-SLAM camera. Only the subset
 * of the librealsense that is useful to robot tracking is exposed in this class.
 *
 *
 * We employ JNI to call librealsense. There *are* Java bindings for librealsense, but they
 * are not complete and do not support our usecase.
 *
 *
 * This class works entirely in 2d, even though the tracking camera supports giving us a third
 * dimension (Z).
 *
 *
 * The coordinate system is as follows: + X == Robot forwards + Y == Robot left (left is from the
 * perspective of a viewer standing behind the robot)
 *
 *
 * All distance units are meters. All time units are seconds.
 */
class T265Camera(
    robotOffsetMeters: Pose,
    odometryCovariance: Double,
    relocMapPath: String,
    appContext: Context?
) {
    enum class PoseConfidence {
        Failed, Low, Medium, High
    }

    class CameraUpdate(pose: Pose, velocity: Pose, confidence: PoseConfidence) {
        /** The robot's pose in meters.  */
        val pose: Pose

        /** The robot's velocity in meters/sec and radians/sec.  */
        val velocity: Pose
        val confidence: PoseConfidence

        init {
            this.pose = pose
            this.velocity = velocity
            this.confidence = confidence
        }
    }

    private val mInitInProgress = AtomicBoolean(false)

    // Protected by mPointerMutex
    private val mPointerMutex = Any()
    private var mHasSeenDeviceBefore = false
    private var mNativeCameraObjectPointer: Long = 0

    /**
     * @return Whether or not the camera is started. Note: the camera driver can still be
     * initializing while it is started.
     */
    // Protected by a mutex on this
    @get:Synchronized
    var isStarted = false
        private set
    private lateinit var mRobotOffset: Pose
    private var mOriginOffset: Pose = Pose()

    // Protected by mUpdateMutex
    private val mUpdateMutex = Any()
    private lateinit var mLastRecievedCameraUpdate: Pose
    private var mLastRecievedUpdate: CameraUpdate? = null
    private lateinit var mPoseConsumer: Consumer<CameraUpdate?>

    // Consumer for CameraUpdate
    private val cameraConsumer: Consumer<Pose> =
        Consumer<Pose> { update: Pose? ->
            synchronized(mUpdateMutex) {
                if (update != null) {
                    mLastRecievedCameraUpdate = update
                }
            }
        }

    /**
     * This method constructs a T265 camera and sets it up with the right info. [ ][T265Camera.start] will not be called, you must call it yourself.
     *
     * @param robotOffset Offset of the center of the robot from the center of the camera.
     * @param odometryCovariance Covariance of the odometry input when doing sensor fusion (you
     * probably want to tune this).
     */
    constructor(robotOffset: Pose, odometryCovariance: Double, appContext: Context?) : this(
        robotOffset,
        odometryCovariance,
        "",
        appContext
    ) {
    }

    /**
     * This method constructs a T265 camera and sets it up with the right info. [ ][T265Camera.start] will not be called, you must call it yourself.
     *
     * @param robotOffsetMeters Offset of the center of the robot from the center of the camera.
     * Units are meters.
     * @param odometryCovariance Covariance of the odometry input when doing sensor fusion (you
     * probably want to tune this)
     * @param relocMapPath path (including filename) to a relocalization map to load.
     */
    init {
        throw mLinkError
        val callback: DeviceListener = object : DeviceListener {
            override fun onDeviceAttach() {
                try {
                    mInitInProgress.set(true)

                    // This check assumes that there's only one camera
                    synchronized(mPointerMutex) {
                        if (mNativeCameraObjectPointer != 0L) {
                            return
                        }
                    }
                    Log.i(
                        kLogTag,
                        "onDeviceAttached called... Will attempt to handoff to native code."
                    )
                    val ptr = newCamera(relocMapPath)
                    synchronized(mPointerMutex) {

                        // newCamera is hoisted out to before we lock because it can block
                        // for a while
                        mHasSeenDeviceBefore = mHasSeenDeviceBefore or (ptr != 0L)
                        mNativeCameraObjectPointer = ptr
                    }
                    setOdometryInfo(
                        robotOffsetMeters.x.toFloat(),
                        robotOffsetMeters.y.toFloat(),
                        robotOffsetMeters.heading.toFloat(),
                        odometryCovariance
                    )
                    mRobotOffset = robotOffsetMeters
                    Log.d(kLogTag, "Native code should be done initializing")
                } catch (e: Exception) {
                    Log.e(
                        kLogTag,
                        "Exception while initializing camera (could be spurious if camera was initially presented as a generic Movidius device as part of the init process)",
                        e
                    )
                } finally {
                    Log.d(kLogTag, "Setting initInProgress to false")
                    mInitInProgress.set(false)
                }
            }

            override fun onDeviceDetach() {
                Log.d(kLogTag, "onDeviceDetach called...")
                synchronized(mPointerMutex) {
                    // Unfortunately we don't get any information about the detaching
                    // device, which means that any rs device detaching will detach *all*
                    // other devices. This is one of the few blockers for multi-device
                    // support.
                    if (mNativeCameraObjectPointer != 0L) {
                        Log.i(
                            kLogTag,
                            "onDeviceDetach called with non-null native pointer. Will attempt to free native objects."
                        )
                        free()
                    }
                }
            }

            override fun equals(obj: Any?): Boolean {
                return false
            }
        }
        Log.d(kLogTag, "Initializing RsContext and asking for permissions...")
        RsContext.init(appContext)
        val rsContext = RsContext()
        rsContext.setDevicesChangedCallback(callback)
        synchronized(mPointerMutex) {
            val numDevices = rsContext.queryDevices().deviceCount
            Log.d(
                kLogTag,
                "Found $numDevices devices at init"
            )
            mHasSeenDeviceBefore = numDevices > 0
        }
    }

    /**
     * This allows the [T265Camera.getLastReceivedCameraUpdate] to start returning pose
     * data. This will NOT reset the camera's pose.
     *
     *
     * This will not restart the camera following [ ][T265Camera.exportRelocalizationMap]. You will have to call [T265Camera.free]
     * and make a new [T265Camera]. This is related to what appears to be a bug in
     * librealsense.
     *
     * @throws RuntimeException This will throw if the camera isn't connected or the camera has
     * already been started.
     */
    fun start() {
        start { update: CameraUpdate? ->
            synchronized(mUpdateMutex) {
                mLastRecievedUpdate = update
            }
        }
    }

    /**
     * This allows the user-provided pose receive callback to receive data. This will NOT reset the
     * camera's pose. This is the advanced version of the start method; if you don't want to provide
     * a callback and just want to call [T265Camera.getLastReceivedCameraUpdate] instead
     * then you should call [T265Camera.start].
     *
     *
     * This will not restart the camera following [ ][T265Camera.exportRelocalizationMap]. You will have to call [T265Camera.free]
     * and make a new [T265Camera]. This is related to what appears to be a bug in
     * librealsense.
     *
     * @param poseConsumer A method to be called every time we receive a pose from *from a
     * different thread*! You must synchronize memory access across threads!
     *
     * Received poses are in meters.
     * @throws RuntimeException This will throw if the camera isn't connected. This will never throw
     * following one successful connection; we will instead continue to try and reconnect.
     */
    @Synchronized
    fun start(poseConsumer: Consumer<CameraUpdate?>?) {
        Log.d(kLogTag, "Trying to start camera callback")
        synchronized(mPointerMutex) {
            if (isStarted) throw RuntimeException("Camera is already started") else if (mNativeCameraObjectPointer == 0L && !mHasSeenDeviceBefore
                && !mInitInProgress.get()
            ) {
                throw RuntimeException("No camera connected")
            }
        }
        synchronized(mUpdateMutex) { mLastRecievedUpdate }
        mPoseConsumer = poseConsumer!!
        isStarted = true
        Log.d(kLogTag, "Camera callback should be started")
    }

    /**
     * Blocks until a new camera update comes in. The camera update will include the latest pose
     * estimate.
     *
     * @return The last received camera update, or null if a custom callback was passed to [     ][T265Camera.start].
     */
    val lastReceivedCameraUpdate: CameraUpdate
        get() {
            synchronized(mUpdateMutex) {
                if (mLastRecievedUpdate == null) {
                    Log.w(
                        kLogTag,
                        "Attempt to get last received update before any updates have been received; are you using the wrong T265Camera::start overload, or is the camera not initialized yet or busy?"
                    )
                    return CameraUpdate(Pose(), Pose(), PoseConfidence.Failed)
                }
                return mLastRecievedUpdate as CameraUpdate
            }
        }

    /** This stops the callback from receiving data, but it does not internally stop the camera.  */
    @Synchronized
    fun stop() {
        Log.d(kLogTag, "Stopping camera callback")
        isStarted = false
    }

    /**
     * Exports a binary relocalization map file to the given path. This will stop the camera.
     * Because of a librealsense bug the camera isn't restarted after you call this method. TODO:
     * Fix that.
     *
     * @param path Path (with filename) to export to
     */
    external fun exportRelocalizationMap(path: String?)

    /**
     * Sends robot velocity as computed from wheel encoders. Note that the X and Y axis orientations
     * are determined by how you set the robotOffset in the constructor.
     *
     * @param velocityXMetersPerSecond The robot-relative velocity along the X axis in meters/sec.
     * @param velocityYMetersPerSecond The robot-relative velocity along the Y axis in meters/sec.
     */
    fun sendOdometry(velocityXMetersPerSecond: Double, velocityYMetersPerSecond: Double) {
        synchronized(mPointerMutex) {
            if (mNativeCameraObjectPointer == 0L) Log.w(
                kLogTag,
                "Can't send odometry while camera is busy or not initialized yet"
            )
        }

        // Only 1 odometry sensor is supported for now (index 0)
        sendOdometryRaw(0, velocityXMetersPerSecond.toFloat(), velocityYMetersPerSecond.toFloat())
    }

    /**
     * This zeroes the camera pose to the provided new pose.
     *
     * @param newPose The pose the camera should be zeroed to.
     */
    @Synchronized
    fun setPose(newPose: Pose) {
        synchronized(mUpdateMutex) {
            mOriginOffset = newPose.relativeTo(
                mLastRecievedCameraUpdate
            )
        }
    }

    /**
     * This will free the underlying native objects. You probably don't want to use this; on program
     * shutdown the native code will gracefully stop and delete any remaining objects.
     */
    external fun free()
    private external fun setOdometryInfo(
        robotOffsetX: Float,
        robotOffsetY: Float,
        robotOffsetRads: Float,
        measurementCovariance: Double
    )

    private external fun sendOdometryRaw(sensorIndex: Int, xVel: Float, yVel: Float)
    private external fun newCamera(mapPath: String): Long
    @Synchronized
    private fun consumeCameraUpdate(
        x: Float, y: Float, radians: Float, dx: Float, dy: Float, dtheta: Float, confOrdinal: Int
    ) {
        val cameraUpdate = Pose(
            x - mRobotOffset.x,
            y - mRobotOffset.y,
            radians.toDouble()
        ).transformBy(mRobotOffset)

        if (!isStarted) return
        cameraConsumer.accept(cameraUpdate)
        consumePoseUpdate(x, y, radians, dx, dy, dtheta, confOrdinal)
    }

    @Synchronized
    private fun consumePoseUpdate(
        x: Float, y: Float, radians: Float, dx: Float, dy: Float, dtheta: Float, confOrdinal: Int
    ) {
        // First we apply an offset to go from the camera coordinate system to the
        // robot coordinate system with an origin at the center of the robot. This
        // is not a directional transformation.
        // Then we transform the pose our camera is giving us so that it reports is
        // the robot's pose, not the camera's. This is a directional transformation.
        val currentPose = Pose(
            x - mRobotOffset.x,
            y - mRobotOffset.y,
            radians.toDouble()
        ).transformBy(mRobotOffset)

        if (!isStarted) return

        // See
        // https://github.com/IntelRealSense/librealsense/blob/7f2ba0de8769620fd672f7b44101f0758e7e2fb3/include/librealsense2/h/rs_types.h#L115
        // for ordinals
        val confidence: PoseConfidence = when (confOrdinal) {
            0x0 -> PoseConfidence.Failed
            0x1 -> PoseConfidence.Low
            0x2 -> PoseConfidence.Medium
            0x3 -> PoseConfidence.High
            else -> throw RuntimeException(
                "Unknown confidence ordinal \""
                        + confOrdinal
                        + "\" passed from native code"
            )
        }
        val transformedPose = mOriginOffset.transformBy(
            Pose(currentPose.vec, currentPose.heading)
        )
        mPoseConsumer.accept(
            CameraUpdate(transformedPose, Pose(dx.toDouble(), dy.toDouble(), dtheta.toDouble()), confidence)
        )
    }

    /** Thrown if something goes wrong in the native code  */
    class CameraJNIException  // This must be static _and_ have this constructor if you want it to be
    // thrown from native code
        (message: String?) : RuntimeException(message)

    companion object {
        private const val kLogTag = "ftc265"
        private lateinit var mLinkError: UnsatisfiedLinkError

        init {
            try {
                Log.d(kLogTag, "Attempting to load native code")
                System.loadLibrary("wrapper")

                // Cleanup is quite tricky for us, because the native code has no idea when Java
                // will be done. (This is why smart pointers don't really make sense in the native
                // code.)
                // Even worse, trying to cleanup with atexit in the native code is too late and
                // results in unfinished callbacks blocking. As a result a shutdown hook is our
                // best option.
                Runtime.getRuntime().addShutdownHook(Thread { cleanup() })
            } catch (e: UnsatisfiedLinkError) {
                Log.e(kLogTag, "Failed to load native code", e)
                mLinkError = e
            }
        }

        private external fun cleanup()
    }

    fun Pose.transformBy(other: Pose): Pose {
        return Pose(
            this.vec.plus(other.vec.rotate(this.heading)),
            atan2(
                cos(this.heading)*cos(other.heading) - sin(this.heading)*sin(other.heading),
                cos(this.heading)*sin(other.heading) - sin(this.heading)*cos(other.heading)
            )
        )
    }

    fun Pose.relativeTo(other: Pose): Pose{
        return Pose(
            other.vec.minus(this.vec).rotate(other.heading.unaryMinus()),
            this.heading.minus(other.heading)
        )
    }
}