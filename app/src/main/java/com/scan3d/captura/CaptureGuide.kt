package com.scan3d.captura

import com.google.ar.core.Pose
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.min
import kotlin.math.sqrt

private fun distance(a: Pose, b: Pose): Float {
    val dx = a.tx() - b.tx()
    val dy = a.ty() - b.ty()
    val dz = a.tz() - b.tz()
    return sqrt(dx * dx + dy * dy + dz * dz)
}

/** Ângulo entre duas orientações, em graus. */
private fun angleDeg(a: Pose, b: Pose): Float {
    val dot = abs(a.qx() * b.qx() + a.qy() * b.qy() + a.qz() * b.qz() + a.qw() * b.qw())
    return Math.toDegrees(2.0 * acos(min(1f, dot).toDouble())).toFloat()
}

/**
 * Decide quando tirar uma foto: após andar ou girar o suficiente, e nunca
 * quando o celular está se movendo rápido demais (foto tremida estraga a
 * reconstrução).
 */
class KeyframeSelector {
    private var lastKeyPose: Pose? = null
    private var lastKeyTime = 0L
    private var prevPose: Pose? = null
    private var prevTime = 0L

    var tooFast = false
        private set

    fun reset() {
        lastKeyPose = null
        prevPose = null
        tooFast = false
    }

    /** Atualiza a velocidade com o quadro atual e diz se é hora de capturar. */
    fun update(pose: Pose, timestampNs: Long): Boolean {
        val prev = prevPose
        if (prev != null && timestampNs > prevTime) {
            val dt = (timestampNs - prevTime) / 1e9f
            val speed = distance(pose, prev) / dt
            val angSpeed = angleDeg(pose, prev) / dt
            tooFast = speed > MAX_SPEED_M_S || angSpeed > MAX_ANG_SPEED_DEG_S
        }
        prevPose = pose
        prevTime = timestampNs
        if (tooFast) return false

        val last = lastKeyPose ?: return true
        if (timestampNs - lastKeyTime < MIN_INTERVAL_NS) return false
        return distance(pose, last) > MIN_TRANSLATION_M || angleDeg(pose, last) > MIN_ROTATION_DEG
    }

    fun markCaptured(pose: Pose, timestampNs: Long) {
        lastKeyPose = pose
        lastKeyTime = timestampNs
    }

    companion object {
        private const val MIN_TRANSLATION_M = 0.15f
        private const val MIN_ROTATION_DEG = 10f
        private const val MIN_INTERVAL_NS = 300_000_000L
        private const val MAX_SPEED_M_S = 0.5f
        private const val MAX_ANG_SPEED_DEG_S = 40f
    }
}

/**
 * Acompanha para onde as fotos já olharam: paredes (dividido em setores ao
 * redor), teto e piso. Serve para avisar o que ainda falta capturar.
 */
class CoverageTracker {
    private val wallSectors = IntArray(SECTORS)
    var ceilingShots = 0
        private set
    var floorShots = 0
        private set
    var pathMeters = 0f
        private set
    private var lastPose: Pose? = null

    fun reset() {
        wallSectors.fill(0)
        ceilingShots = 0
        floorShots = 0
        pathMeters = 0f
        lastPose = null
    }

    fun addKeyframe(pose: Pose) {
        lastPose?.let { pathMeters += distance(pose, it) }
        lastPose = pose
        // Direção do olhar = -Z da câmera, no mundo.
        val z = pose.zAxis
        val fx = -z[0]
        val fy = -z[1]
        val fz = -z[2]
        val pitch = Math.toDegrees(asin(fy.coerceIn(-1f, 1f).toDouble()))
        when {
            pitch > 45 -> ceilingShots++
            pitch < -45 -> floorShots++
            else -> {
                val yaw = Math.toDegrees(atan2(fx.toDouble(), -fz.toDouble()))
                val sector = (((yaw + 360.0) % 360.0) / (360.0 / SECTORS)).toInt()
                    .coerceIn(0, SECTORS - 1)
                wallSectors[sector]++
            }
        }
    }

    val wallCoveragePercent: Int
        get() = wallSectors.count { it >= MIN_SHOTS_PER_SECTOR } * 100 / SECTORS

    /** Mensagem curta do que ainda falta. */
    fun hint(): String {
        val missing = mutableListOf<String>()
        if (wallCoveragePercent < 100) missing += "paredes ao redor"
        if (ceilingShots < MIN_CEILING_FLOOR) missing += "teto"
        if (floorShots < MIN_CEILING_FLOOR) missing += "piso"
        return if (missing.isEmpty()) "Cobertura básica completa. Capture cantos, portas e detalhes."
        else "Falta: " + missing.joinToString(", ")
    }

    companion object {
        private const val SECTORS = 24
        private const val MIN_SHOTS_PER_SECTOR = 3
        private const val MIN_CEILING_FLOOR = 8
    }
}
