package org.firstinspires.ftc.teamcode.cameras

import com.qualcomm.robotcore.hardware.HardwareDevice
import org.intel.rs.Context
import org.intel.rs.frame.DepthFrame
import org.intel.rs.pipeline.Config
import org.intel.rs.pipeline.Pipeline
import org.intel.rs.types.Format
import org.intel.rs.types.Pose
import org.intel.rs.types.Stream

class DepthCamera {

    lateinit var depth: DepthFrame
    private val context = Context()
    private val pipeline = Pipeline(context)

    init {
        val config = Config()
        config.enableStream(Stream.Depth, Format.Z16)

        pipeline.start(config)
    }

    private fun getStreams() {
        val frames = pipeline.waitForFrames()
        val depthFrame = frames.depthFrame

    }
}