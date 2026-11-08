package com.scan3d.captura

import android.media.Image
import java.nio.ByteBuffer
import java.nio.ByteOrder

object YuvUtil {

    /** Converte uma imagem YUV_420_888 do ARCore para NV21 (Y + VU intercalado). */
    fun toNv21(image: Image): ByteArray {
        val w = image.width
        val h = image.height
        val out = ByteArray(w * h * 3 / 2)
        val y = image.planes[0]
        val u = image.planes[1]
        val v = image.planes[2]
        copyPlane(y.buffer, y.rowStride, y.pixelStride, w, h, out, 0, 1)
        copyPlane(v.buffer, v.rowStride, v.pixelStride, w / 2, h / 2, out, w * h, 2)
        copyPlane(u.buffer, u.rowStride, u.pixelStride, w / 2, h / 2, out, w * h + 1, 2)
        return out
    }

    private fun copyPlane(
        buffer: ByteBuffer, rowStride: Int, pixelStride: Int,
        width: Int, height: Int, out: ByteArray, offset: Int, outStride: Int,
    ) {
        val src = buffer.duplicate()
        if (pixelStride == 1 && outStride == 1) {
            // Caminho rápido: copia linha a linha.
            for (row in 0 until height) {
                src.position(row * rowStride)
                src.get(out, offset + row * width, width)
            }
            return
        }
        var o = offset
        for (row in 0 until height) {
            val rowStart = row * rowStride
            for (col in 0 until width) {
                out[o] = src.get(rowStart + col * pixelStride)
                o += outStride
            }
        }
    }

    /** Copia a profundidade DEPTH16 do ARCore (milímetros) para um array compacto. */
    fun depthToShorts(depth: Image): ShortArray {
        val plane = depth.planes[0]
        val w = depth.width
        val h = depth.height
        val buf = plane.buffer.duplicate().order(ByteOrder.nativeOrder())
        val out = ShortArray(w * h)
        for (row in 0 until h) {
            val rowStart = row * plane.rowStride
            for (col in 0 until w) {
                out[row * w + col] = buf.getShort(rowStart + col * plane.pixelStride)
            }
        }
        return out
    }
}
