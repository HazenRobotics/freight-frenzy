package org.firstinspires.ftc.teamcode.localization

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import org.firstinspires.ftc.teamcode.cameras.TrackingCamera
import org.intel.rs.types.Pose
import kotlin.math.atan2
import kotlin.math.pow

class TrackingCameraLocalizer : Localizer {

	override lateinit var poseEstimate: Pose2d
	override lateinit var poseVelocity: Pose2d
	lateinit var poseAcceleration: Pose2d

	private val trackingCam = TrackingCamera()

	override fun update( ) {
		trackingCam.getStream()
		poseEstimate = camPoseToPose2d(trackingCam.pose)
		poseVelocity = camVeloToPose2d(trackingCam.pose)
		poseAcceleration = camAccToPose2d(trackingCam.pose)
	}

	private fun camPoseToPose2d(camValue: Pose): Pose2d {
		return Pose2d(
				camValue.translation[0].toDouble(),
				camValue.translation[1].toDouble(),
				quaternionToEulerAngle(camValue.rotation)
		)
	}

	private fun camVeloToPose2d(camValue: Pose): Pose2d {
		return Pose2d(
				camValue.velocity[0].toDouble(),
				camValue.velocity[0].toDouble(),
				camValue.angular_velocity[2].toDouble()
		)
	}

	private fun camAccToPose2d(camValue: Pose): Pose2d {
		return Pose2d(
				camValue.acceleration[0].toDouble(),
				camValue.acceleration[1].toDouble(),
				camValue.acceleration[2].toDouble()
		)
	}

	private fun quaternionToEulerAngle(q: FloatArray): Double {
		val top = 2 * (q[0] * q[3] + q[1] * q[2])
		val bottom = 1 - 2 * (q[2].pow(2) + q[3].pow(2))
		return atan2(top, bottom).toDouble()
	}
}