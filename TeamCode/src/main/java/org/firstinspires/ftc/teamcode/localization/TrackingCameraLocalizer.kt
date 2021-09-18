package org.firstinspires.ftc.teamcode.localization

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import org.firstinspires.ftc.teamcode.cameras.TrackingCamera
import org.intel.rs.types.Pose

class TrackingCameraLocalizer : Localizer {

	override lateinit var poseEstimate: Pose2d
	override lateinit var poseVelocity: Pose2d

	private val trackingCam = TrackingCamera()

	override fun update( ) {
		poseEstimate = camPoseToPose2d(trackingCam.pose)
		poseVelocity = camVeloToPose2d(trackingCam.pose)
	}

	private fun camPoseToPose2d(camValue: Pose): Pose2d {
		return Pose2d(
				camValue.translation[0].toDouble(),
				camValue.translation[1].toDouble(), /*TODO: need to figure out quaternions to axis angles*/
		)
	}

	private fun camVeloToPose2d(camValue: Pose): Pose2d {
		return Pose2d(
				camValue.velocity[0].toDouble(),
				camValue.velocity[0].toDouble(),
				camValue.angular_velocity[2].toDouble()
		)
	}
}