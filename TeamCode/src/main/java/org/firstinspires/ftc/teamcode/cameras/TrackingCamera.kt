package org.firstinspires.ftc.teamcode.cameras

import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.intel.rs.Context
import org.intel.rs.device.Device
import org.intel.rs.device.DeviceList
import org.intel.rs.pipeline.Config
import org.intel.rs.pipeline.Pipeline
import org.intel.rs.types.Format
import org.intel.rs.types.Pose
import org.intel.rs.types.Stream
import org.intel.rs.frame.FrameList




class TrackingCamera {

    lateinit var pose: Pose
    private val context = Context()
    private val pipeline = Pipeline(context)

    init {
        val config = Config()
        config.enableStream(Stream.Pose, Format.Xyz32f)

        pipeline.start(config)
    }

    private fun getStreams() {
        val frames = pipeline.waitForFrames()
        val poseFrame = frames.poseFrame
        poseFrame.pose
    }



}