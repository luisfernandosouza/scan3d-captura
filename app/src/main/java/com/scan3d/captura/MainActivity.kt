package com.scan3d.captura

import android.Manifest
import android.annotation.SuppressLint
import android.content.Intent
import android.content.pm.PackageManager
import android.media.Image
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.os.Bundle
import android.os.SystemClock
import android.text.InputType
import android.view.MotionEvent
import android.view.Surface
import android.view.View
import android.view.WindowManager
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.core.content.FileProvider
import androidx.core.view.ViewCompat
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import com.google.ar.core.ArCoreApk
import com.google.ar.core.CameraConfig
import com.google.ar.core.CameraConfigFilter
import com.google.ar.core.Config
import com.google.ar.core.DepthPoint
import com.google.ar.core.Frame
import com.google.ar.core.Plane
import com.google.ar.core.Point
import com.google.ar.core.Session
import com.google.ar.core.TrackingFailureReason
import com.google.ar.core.TrackingState
import com.google.ar.core.exceptions.NotYetAvailableException
import com.google.ar.core.exceptions.UnavailableException
import org.json.JSONObject
import java.io.File
import java.util.concurrent.ConcurrentLinkedQueue
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.sqrt

class MainActivity : AppCompatActivity(), GLSurfaceView.Renderer {

    private lateinit var surfaceView: GLSurfaceView
    private lateinit var statusText: TextView
    private lateinit var coverageText: TextView
    private lateinit var recordButton: Button
    private lateinit var measureButton: Button
    private lateinit var shareButton: Button

    private var session: Session? = null
    private var installRequested = false
    private var depthSupported = false
    private var imageWidth = 0
    private var imageHeight = 0

    private val background = BackgroundRenderer()
    private var viewportChanged = false
    private var viewportWidth = 0
    private var viewportHeight = 0

    private lateinit var writer: CaptureWriter
    private val keyframes = KeyframeSelector()
    private val coverage = CoverageTracker()

    @Volatile private var recording = false
    @Volatile private var measuring = false
    private var frameIndex = 0
    private val measurePoints = mutableListOf<FloatArray>()
    private val pendingTaps = ConcurrentLinkedQueue<FloatArray>()
    private var lastSessionDir: File? = null
    private var lastUiUpdate = 0L
    private val recentSharpness = ArrayDeque<Double>()
    private var blurryCount = 0

    private val cameraPermission =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
            if (!granted) {
                Toast.makeText(this, "A câmera é necessária para escanear.", Toast.LENGTH_LONG).show()
                finish()
            }
        }

    @SuppressLint("ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        applySafeArea()
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        surfaceView = findViewById(R.id.surfaceView)
        statusText = findViewById(R.id.statusText)
        coverageText = findViewById(R.id.coverageText)
        recordButton = findViewById(R.id.recordButton)
        measureButton = findViewById(R.id.measureButton)
        shareButton = findViewById(R.id.shareButton)

        writer = CaptureWriter(getExternalFilesDir(null) ?: filesDir)

        surfaceView.preserveEGLContextOnPause = true
        surfaceView.setEGLContextClientVersion(2)
        surfaceView.setEGLConfigChooser(8, 8, 8, 8, 16, 0)
        surfaceView.setRenderer(this)
        surfaceView.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
        surfaceView.setOnTouchListener { _, event ->
            if (event.action == MotionEvent.ACTION_UP && measuring) {
                pendingTaps.add(floatArrayOf(event.x, event.y))
            }
            true
        }

        recordButton.setOnClickListener { if (recording) stopRecording() else startRecording() }
        measureButton.setOnClickListener { startMeasuring() }
        shareButton.setOnClickListener { exportLastSession() }
    }

    /**
     * Desenha a câmera na tela inteira, mas mantém os textos e botões fora da
     * barra de status, do recorte da câmera frontal e da barra de navegação.
     */
    private fun applySafeArea() {
        WindowCompat.setDecorFitsSystemWindows(window, false)
        val top = findViewById<View>(R.id.topPanel)
        val bottom = findViewById<View>(R.id.bottomPanel)
        val pad = (12 * resources.displayMetrics.density).toInt()
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(android.R.id.content)) { _, insets ->
            val bars = insets.getInsets(
                WindowInsetsCompat.Type.systemBars() or WindowInsetsCompat.Type.displayCutout()
            )
            top.setPadding(pad + bars.left, pad + bars.top, pad + bars.right, pad)
            bottom.setPadding(pad + bars.left, pad, pad + bars.right, pad + bars.bottom)
            insets
        }
    }

    override fun onResume() {
        super.onResume()
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
            != PackageManager.PERMISSION_GRANTED
        ) {
            cameraPermission.launch(Manifest.permission.CAMERA)
            return
        }
        if (session == null) {
            try {
                when (ArCoreApk.getInstance().requestInstall(this, !installRequested)) {
                    ArCoreApk.InstallStatus.INSTALL_REQUESTED -> {
                        installRequested = true
                        return
                    }
                    ArCoreApk.InstallStatus.INSTALLED -> Unit
                    else -> Unit
                }
                session = Session(this).also { configureSession(it) }
            } catch (e: UnavailableException) {
                statusText.text = "ARCore indisponível neste aparelho: ${e.javaClass.simpleName}"
                return
            } catch (e: Exception) {
                statusText.text = "Erro ao iniciar o ARCore: ${e.message}"
                return
            }
        }
        try {
            session?.resume()
        } catch (e: Exception) {
            statusText.text = "Não foi possível abrir a câmera: ${e.message}"
            session = null
            return
        }
        surfaceView.onResume()
    }

    override fun onPause() {
        super.onPause()
        // Ao pausar, o ARCore perde o sistema de coordenadas: a sessão precisa terminar aqui.
        if (recording) stopRecording()
        surfaceView.onPause()
        session?.pause()
    }

    override fun onDestroy() {
        super.onDestroy()
        session?.close()
        session = null
        writer.shutdown()
    }

    private fun configureSession(s: Session) {
        // Usa a maior resolução de imagem que o ARCore oferece neste aparelho.
        val filter = CameraConfigFilter(s).setFacingDirection(CameraConfig.FacingDirection.BACK)
        val best = s.getSupportedCameraConfigs(filter)
            .maxByOrNull { it.imageSize.width * it.imageSize.height }
        if (best != null) {
            s.cameraConfig = best
            imageWidth = best.imageSize.width
            imageHeight = best.imageSize.height
        }
        depthSupported = s.isDepthModeSupported(Config.DepthMode.AUTOMATIC)
        val config = Config(s).apply {
            // Foco automático: o rastreamento do ARCore fica mais estável.
            // A fotogrametria no PC recalcula a lente de qualquer forma.
            focusMode = Config.FocusMode.AUTO
            updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
            planeFindingMode = Config.PlaneFindingMode.HORIZONTAL_AND_VERTICAL
            lightEstimationMode = Config.LightEstimationMode.DISABLED
            depthMode = if (depthSupported) Config.DepthMode.AUTOMATIC else Config.DepthMode.DISABLED
        }
        s.configure(config)
    }

    // ---------------------------------------------------------------- gravação

    private fun startRecording() {
        if (session == null) return
        val info = JSONObject().apply {
            put("image_width", imageWidth)
            put("image_height", imageHeight)
            put("depth_supported", depthSupported)
            put("arcore_apk_version", arcoreVersion())
        }
        lastSessionDir = writer.start(info)
        frameIndex = 0
        recentSharpness.clear()
        blurryCount = 0
        keyframes.reset()
        coverage.reset()
        recording = true
        recordButton.text = "Parar"
        measureButton.isEnabled = true
        shareButton.isEnabled = false
    }

    private fun stopRecording() {
        recording = false
        measuring = false
        measurePoints.clear()
        recordButton.isEnabled = false
        measureButton.isEnabled = false
        statusText.text = "Salvando fotos…"
        Thread {
            val dir = writer.stop()
            runOnUiThread {
                recordButton.isEnabled = true
                recordButton.text = "Iniciar"
                shareButton.isEnabled = dir != null
                statusText.text = "Captura salva: ${writer.count} fotos"
            }
        }.start()
    }

    private fun arcoreVersion(): String = try {
        packageManager.getPackageInfo("com.google.ar.core", 0).versionName ?: "?"
    } catch (e: Exception) {
        "?"
    }

    // ------------------------------------------------------- medida de trena

    private fun startMeasuring() {
        measurePoints.clear()
        measuring = true
        Toast.makeText(
            this,
            "Toque no 1º ponto (ex.: um lado do batente da porta)",
            Toast.LENGTH_LONG
        ).show()
    }

    /** Roda na thread de OpenGL: converte toques em pontos 3D. */
    private fun handleTaps(frame: Frame) {
        while (true) {
            val tap = pendingTaps.poll() ?: break
            if (!measuring) continue
            val hit = frame.hitTest(tap[0], tap[1]).firstOrNull { h ->
                when (val t = h.trackable) {
                    is Plane -> t.isPoseInPolygon(h.hitPose)
                    is DepthPoint -> true
                    is Point -> t.orientationMode == Point.OrientationMode.ESTIMATED_SURFACE_NORMAL
                    else -> false
                }
            }
            if (hit == null) {
                runOnUiThread {
                    Toast.makeText(this, "Superfície não encontrada. Aponte para um ponto com textura e toque de novo.", Toast.LENGTH_SHORT).show()
                }
                continue
            }
            val p = hit.hitPose.translation
            measurePoints.add(p)
            if (measurePoints.size == 1) {
                runOnUiThread {
                    Toast.makeText(this, "Ponto 1 marcado. Toque no 2º ponto.", Toast.LENGTH_SHORT).show()
                }
            } else {
                val a = measurePoints[0]
                val b = measurePoints[1]
                val dx = a[0] - b[0]; val dy = a[1] - b[1]; val dz = a[2] - b[2]
                val meters = sqrt((dx * dx + dy * dy + dz * dz).toDouble())
                measuring = false
                measurePoints.clear()
                runOnUiThread { askRealMeasure(a, b, meters) }
            }
        }
    }

    private fun askRealMeasure(a: FloatArray, b: FloatArray, arcoreMeters: Double) {
        val input = EditText(this).apply {
            inputType = InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL
            hint = "Medida real em cm"
        }
        AlertDialog.Builder(this)
            .setTitle("Medida de referência")
            .setMessage(
                "O ARCore mediu %.1f cm.\nDigite a medida real feita com a trena (cm).".format(arcoreMeters * 100)
            )
            .setView(input)
            .setPositiveButton("Salvar") { _, _ ->
                val cm = input.text.toString().replace(',', '.').toDoubleOrNull()
                if (cm != null && cm > 0 && recording) {
                    val ratio = (cm / 100.0) / arcoreMeters
                    if (ratio < 0.7 || ratio > 1.4) {
                        Toast.makeText(
                            this,
                            "Medida muito diferente da do ARCore (rastreamento perdido?). Não foi salva: tente de novo.",
                            Toast.LENGTH_LONG
                        ).show()
                    } else {
                        writer.addCalibration(a, b, arcoreMeters, cm / 100.0)
                        Toast.makeText(this, "Medida salva.", Toast.LENGTH_SHORT).show()
                    }
                }
            }
            .setNegativeButton("Pular", null)
            .show()
    }

    // ---------------------------------------------------------- exportação

    private fun exportLastSession() {
        val dir = lastSessionDir ?: return
        shareButton.isEnabled = false
        statusText.text = "Compactando captura…"
        Thread {
            val zip = try {
                writer.zipSession(dir)
            } catch (e: Exception) {
                null
            }
            runOnUiThread {
                shareButton.isEnabled = true
                if (zip == null) {
                    statusText.text = "Falha ao compactar a captura."
                    return@runOnUiThread
                }
                statusText.text = "Pronto: ${zip.name}"
                val uri = FileProvider.getUriForFile(this, "$packageName.fileprovider", zip)
                val send = Intent(Intent.ACTION_SEND).apply {
                    type = "application/zip"
                    putExtra(Intent.EXTRA_STREAM, uri)
                    addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
                }
                startActivity(Intent.createChooser(send, "Enviar captura"))
            }
        }.start()
    }

    // ------------------------------------------------------------- OpenGL

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0f, 0f, 0f, 1f)
        background.createOnGlThread()
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        viewportWidth = width
        viewportHeight = height
        viewportChanged = true
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        val s = session ?: return
        if (viewportChanged) {
            s.setDisplayGeometry(Surface.ROTATION_0, viewportWidth, viewportHeight)
            viewportChanged = false
        }
        s.setCameraTextureName(background.textureId)
        val frame = try {
            s.update()
        } catch (e: Exception) {
            return
        }
        background.draw(frame)
        handleTaps(frame)

        val camera = frame.camera
        if (camera.trackingState != TrackingState.TRACKING) {
            postStatus(trackingMessage(camera.trackingFailureReason), null)
            return
        }

        val pose = camera.pose
        val shouldCapture = keyframes.update(pose, frame.timestamp)
        if (recording && shouldCapture && !writer.isBusy) {
            if (capture(frame)) {
                keyframes.markCaptured(pose, frame.timestamp)
                coverage.addKeyframe(pose)
            }
        }

        val status = when {
            !recording -> "Pronto. Toque em Iniciar e ande devagar pelo ambiente."
            measuring -> "Modo medida: toque nos dois pontos medidos com a trena."
            keyframes.unstable -> "Estabilizando rastreamento… aponte para áreas com detalhes."
            keyframes.tooFast -> "Mais devagar! Movimento rápido borra as fotos."
            else -> "Gravando: ${writer.count} fotos" +
                if (blurryCount > 0) " ($blurryCount tremidas descartadas)" else ""
        }
        val cov = if (recording) {
            "Paredes: ${coverage.wallCoveragePercent}% · Teto: ${coverage.ceilingShots} · " +
                "Piso: ${coverage.floorShots} · Percurso: %.1f m\n".format(coverage.pathMeters) +
                coverage.hint()
        } else null
        postStatus(status, cov)
    }

    /** Copia a imagem da câmera (e a profundidade) e envia para gravação. */
    private fun capture(frame: Frame): Boolean {
        val camera = frame.camera
        val image: Image = try {
            frame.acquireCameraImage()
        } catch (e: NotYetAvailableException) {
            return false
        } catch (e: Exception) {
            return false
        }
        val w = image.width
        val h = image.height
        val nv21 = try {
            YuvUtil.toNv21(image)
        } finally {
            image.close()
        }

        // Descarta fotos tremidas: bem menos nítidas que as últimas aceitas.
        val sharp = YuvUtil.sharpness(nv21, w, h)
        val reference = recentSharpness.sorted().getOrNull(recentSharpness.size / 2)
        recentSharpness.addLast(sharp)
        if (recentSharpness.size > 15) recentSharpness.removeFirst()
        if (reference != null && recentSharpness.size >= 5 && sharp < 0.6 * reference) {
            blurryCount++
            return false
        }

        var depth: ShortArray? = null
        var dw = 0
        var dh = 0
        if (depthSupported) {
            try {
                frame.acquireDepthImage16Bits().use { d ->
                    dw = d.width
                    dh = d.height
                    depth = YuvUtil.depthToShorts(d)
                }
            } catch (e: NotYetAvailableException) {
                // Sem profundidade neste quadro; a foto ainda serve.
            } catch (e: Exception) {
            }
        }

        val intr = camera.imageIntrinsics
        val focal = intr.focalLength
        val pp = intr.principalPoint
        val dims = intr.imageDimensions
        // Os intrínsecos se referem a "dims"; ajusta se a imagem tiver outro tamanho.
        val sx = w.toFloat() / dims[0]
        val sy = h.toFloat() / dims[1]
        val pose = camera.pose

        writer.submit(
            KeyframeData(
                index = frameIndex++,
                timestampNs = frame.timestamp,
                translation = pose.translation,
                rotation = pose.rotationQuaternion,
                fx = focal[0] * sx, fy = focal[1] * sy,
                cx = pp[0] * sx, cy = pp[1] * sy,
                nv21 = nv21, width = w, height = h,
                depthMm = depth, depthWidth = dw, depthHeight = dh,
            )
        )
        return true
    }

    private fun postStatus(status: String, cov: String?) {
        val now = SystemClock.uptimeMillis()
        if (now - lastUiUpdate < 200) return
        lastUiUpdate = now
        runOnUiThread {
            statusText.text = status
            coverageText.text = cov ?: ""
        }
    }

    private fun trackingMessage(reason: TrackingFailureReason): String = when (reason) {
        TrackingFailureReason.NONE -> "Inicializando: mova o celular devagar."
        TrackingFailureReason.BAD_STATE -> "Erro de rastreamento. Reinicie a captura."
        TrackingFailureReason.INSUFFICIENT_LIGHT -> "Pouca luz. Acenda as luzes do ambiente."
        TrackingFailureReason.EXCESSIVE_MOTION -> "Movimento rápido demais. Vá mais devagar."
        TrackingFailureReason.INSUFFICIENT_FEATURES -> "Superfície sem textura. Aponte para áreas com mais detalhes."
        TrackingFailureReason.CAMERA_UNAVAILABLE -> "Câmera em uso por outro app."
        else -> "Rastreamento perdido. Volte para uma área já capturada."
    }
}
