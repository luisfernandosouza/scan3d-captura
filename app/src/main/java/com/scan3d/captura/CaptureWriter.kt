package com.scan3d.captura

import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.os.Build
import org.json.JSONArray
import org.json.JSONObject
import java.io.BufferedOutputStream
import java.io.File
import java.io.FileOutputStream
import java.io.FileWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicInteger
import java.util.zip.ZipEntry
import java.util.zip.ZipOutputStream

/** Dados de um quadro-chave, já copiados da câmera (o Image original já foi liberado). */
class KeyframeData(
    val index: Int,
    val timestampNs: Long,
    /** Posição da câmera no mundo do ARCore, em metros. */
    val translation: FloatArray,
    /** Rotação da câmera (qx, qy, qz, qw). */
    val rotation: FloatArray,
    val fx: Float, val fy: Float, val cx: Float, val cy: Float,
    val nv21: ByteArray, val width: Int, val height: Int,
    /** Profundidade em milímetros (uint16), ou null se indisponível. */
    val depthMm: ShortArray?, val depthWidth: Int, val depthHeight: Int,
)

/**
 * Grava uma sessão de captura em disco:
 *
 *   capturas/AAAAMMDD_HHMMSS/
 *     session.json      metadados da sessão e convenções
 *     frames.jsonl      uma linha JSON por foto (pose + intrínsecos)
 *     images/000000.jpg fotos da câmera
 *     depth/000000.u16  profundidade bruta (uint16 little-endian, mm), se houver
 *     calibration.json  medidas de trena para corrigir a escala
 */
class CaptureWriter(private val baseDir: File) {

    private val executor = Executors.newSingleThreadExecutor()
    private val pending = AtomicInteger(0)

    var sessionDir: File? = null
        private set
    private var framesWriter: FileWriter? = null
    private val calibrations = JSONArray()
    private var frameCount = 0
    private var startedAt = 0L
    private var sessionInfo = JSONObject()

    /** Evita acumular fotos na memória quando o disco está lento. */
    val isBusy: Boolean get() = pending.get() >= MAX_PENDING

    fun start(info: JSONObject): File {
        val name = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val dir = File(baseDir, "capturas/$name")
        File(dir, "images").mkdirs()
        File(dir, "depth").mkdirs()
        sessionDir = dir
        frameCount = 0
        startedAt = System.currentTimeMillis()
        sessionInfo = info
        while (calibrations.length() > 0) calibrations.remove(0)
        framesWriter = FileWriter(File(dir, "frames.jsonl"))
        writeSessionJson(finished = false)
        return dir
    }

    fun submit(frame: KeyframeData) {
        val dir = sessionDir ?: return
        val writer = framesWriter ?: return
        pending.incrementAndGet()
        frameCount++
        executor.execute {
            try {
                val name = "%06d".format(frame.index)
                val yuv = YuvImage(frame.nv21, ImageFormat.NV21, frame.width, frame.height, null)
                FileOutputStream(File(dir, "images/$name.jpg")).use { out ->
                    yuv.compressToJpeg(Rect(0, 0, frame.width, frame.height), JPEG_QUALITY, out)
                }
                var depthFile: String? = null
                frame.depthMm?.let { depth ->
                    depthFile = "depth/$name.u16"
                    BufferedOutputStream(FileOutputStream(File(dir, depthFile!!))).use { out ->
                        for (v in depth) {
                            out.write(v.toInt() and 0xFF)
                            out.write((v.toInt() shr 8) and 0xFF)
                        }
                    }
                }
                val json = JSONObject().apply {
                    put("index", frame.index)
                    put("image", "images/$name.jpg")
                    put("timestamp_ns", frame.timestampNs)
                    put("t", JSONArray(frame.translation.map { it.toDouble() }))
                    put("q_xyzw", JSONArray(frame.rotation.map { it.toDouble() }))
                    put("fx", frame.fx.toDouble()); put("fy", frame.fy.toDouble())
                    put("cx", frame.cx.toDouble()); put("cy", frame.cy.toDouble())
                    put("width", frame.width); put("height", frame.height)
                    if (depthFile != null) {
                        put("depth", depthFile)
                        put("depth_width", frame.depthWidth)
                        put("depth_height", frame.depthHeight)
                    }
                }
                synchronized(writer) {
                    writer.write(json.toString())
                    writer.write("\n")
                    writer.flush()
                }
            } finally {
                pending.decrementAndGet()
            }
        }
    }

    /** Guarda uma medida de referência: dois pontos 3D e o valor real medido com trena. */
    fun addCalibration(p1: FloatArray, p2: FloatArray, arcoreMeters: Double, realMeters: Double) {
        calibrations.put(JSONObject().apply {
            put("p1", JSONArray(p1.map { it.toDouble() }))
            put("p2", JSONArray(p2.map { it.toDouble() }))
            put("arcore_m", arcoreMeters)
            put("real_m", realMeters)
        })
        val dir = sessionDir ?: return
        executor.execute {
            File(dir, "calibration.json").writeText(
                JSONObject().put("measurements", calibrations).toString(2)
            )
        }
    }

    /** Encerra a sessão e espera todas as fotos serem gravadas. */
    fun stop(): File? {
        val dir = sessionDir ?: return null
        executor.execute {
            framesWriter?.close()
            framesWriter = null
            writeSessionJson(finished = true)
        }
        // Barreira: espera a fila esvaziar.
        executor.submit {}.get(60, TimeUnit.SECONDS)
        return dir
    }

    val count: Int get() = frameCount

    private fun writeSessionJson(finished: Boolean) {
        val dir = sessionDir ?: return
        val json = JSONObject(sessionInfo.toString()).apply {
            put("format_version", 1)
            put("app_version", BuildConfigInfo.VERSION)
            put("device", "${Build.MANUFACTURER} ${Build.MODEL}")
            put("android_sdk", Build.VERSION.SDK_INT)
            put("started_at_ms", startedAt)
            put("finished", finished)
            put("frame_count", frameCount)
            put(
                "pose_convention",
                "ARCore Camera.getPose(): mundo em metros, Y para cima. Câmera OpenGL: " +
                    "+X direita, +Y cima, -Z direção do olhar, relativo à imagem gravada. " +
                    "t = posição da câmera no mundo, q_xyzw = rotação câmera->mundo."
            )
        }
        File(dir, "session.json").writeText(json.toString(2))
    }

    /** Compacta a sessão num .zip para enviar ao computador. Roda fora da thread de UI. */
    fun zipSession(dir: File): File {
        val zip = File(dir.parentFile, "${dir.name}.zip")
        ZipOutputStream(BufferedOutputStream(FileOutputStream(zip))).use { zos ->
            dir.walkTopDown().filter { it.isFile }.forEach { file ->
                val entryName = "${dir.name}/" + file.relativeTo(dir).path.replace('\\', '/')
                val entry = ZipEntry(entryName)
                // JPEGs já são comprimidos: armazenar sem recompressão é mais rápido.
                if (file.extension == "jpg") {
                    entry.method = ZipEntry.STORED
                    entry.size = file.length()
                    entry.crc = crc32(file)
                }
                zos.putNextEntry(entry)
                file.inputStream().use { it.copyTo(zos) }
                zos.closeEntry()
            }
        }
        return zip
    }

    private fun crc32(file: File): Long {
        val crc = java.util.zip.CRC32()
        file.inputStream().use { input ->
            val buf = ByteArray(64 * 1024)
            while (true) {
                val n = input.read(buf)
                if (n < 0) break
                crc.update(buf, 0, n)
            }
        }
        return crc.value
    }

    fun shutdown() {
        executor.shutdown()
    }

    companion object {
        private const val MAX_PENDING = 4
        private const val JPEG_QUALITY = 95
    }
}

object BuildConfigInfo {
    const val VERSION = "0.1.1"
}
