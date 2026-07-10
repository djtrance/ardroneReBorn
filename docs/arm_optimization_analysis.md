# ARM Optimization Analysis for OMAP3530 Optical Flow

## Pipeline Actual

```
V4L2 UYVY frame ──→ uyvy_to_nv12() ──→ memcpy a DSP ──→ H.264 encode
                          │
                     full frame copy
                  (width×height×1.5 bytes)
```

**Problema**: `uyvy_to_nv12()` copia bytes y convierte formato completo (1.5× la data del frame). Para optical flow necesitamos Y (luma), no NV12 completo. El canal Y ya existe en UYVY en posiciones impares sin necesidad de conversion.

---

## Análisis del Canal Y en UYVY

Formato UYVY (4 bytes por 2 pixeles):

```
Byte:   0   1   2   3   4   5   6   7
Pixel:  U0  Y0  V0  Y1  U2  Y2  V2  Y3
```

- Y samples en offsets: 1, 3, 5, 7... (`frame_data + 1 + x*2`)
- Row stride (tight): `width * 2` bytes por fila

**Zero-copy Y access**: `uint8_t* y = frame_data + 1; y_pixel(x,y) = y[y * bytesperline + x * 2]`

Esto evita por completo la conversion UYVY→NV12 (ahorro de ~1.5× ancho de banda de memoria por frame, mas la CPU de la conversion).

---

## Paparazzi OpticFlow — Acceso a Pixeles

| Algoritmo | `image_t` stride? | Acceso UYVY Y? | Acceso esperado |
|-----------|------------------|----------------|-----------------|
| **FAST9** | No (tight packing) | Sí (+1 offset con pixel_size=2) | `buf[y*w*ps + x*ps + ps/2]` |
| **Lucas-Kanade** | No | No (requiere grayscale) | `buf[y*w*ps + x*ps]` con ps=1 |
| **Edge Flow** | No | Parcial (lee U/V en offset 0, NO Y) | `interlace * (y*w + x)` bug potencial |

**Limitacion critica**: Todos asumen `row_stride = w * pixel_size`. Si V4L2 devuelve `bytesperline > w * 2` (padding por alineacion de hardware), los algoritmos fallan. El AR.Drone 2.0 típicamente usa tight packing a 1280×720 UYVY.

**FAST funciona directamente sobre UYVY** — extrae Y automaticamente. LK necesita conversion a grayscale primero (Paparazzi ya tiene `image_to_grayscale()`).

---

## Optimizaciones ARM Evaluadas

### ARM CMSIS-CV (`github.com/ARM-software/CMSIS-CV`)

| Funcion | Entrada | Salida | Util para OF? |
|---------|---------|--------|---------------|
| `arm_sobel_horizontal` | gray8 | q15 gradient | Sí (gradiente espacial Ix) |
| `arm_sobel_vertical` | gray8 | q15 gradient | Sí (gradiente espacial Iy) |
| `arm_gaussian_filter_3x3/5x5` | gray8 | gray8 | Sí (pre-suavizado LK) |
| `arm_canny_edge_sobel` | gray8 | gray8 | Referencia (gradiente interno) |
| `arm_image_resize_gray8` | gray8 | gray8 | Piramide (pero float, no ideal) |
| `arm_yuv420_to_gray8` | yuv420 planar | gray8 | No (espera planar, no UYVY) |

**Conclusion**: CMSIS-CV no tiene optical flow, corner detection (FAST/Harris), ni piramide. Lo util son los filtros Sobel/Gaussian como building blocks. Apunta a Cortex-M (Helium), el soporte Neon es experimental.

### ARM CMSIS-DSP (`github.com/ARM-software/CMSIS-DSP`)

| Modulo | Utilidad para OF |
|--------|-----------------|
| Matrix operations (2×2 solve) | Solucion sistema LK: `[ΣIx² ΣIxIy; ΣIxIy ΣIy²] * v = -[ΣIxIt; ΣIyIt]` |
| FFT | Analisis frecuencia, correlation |
| Statistics (mean, std) | Normalizacion ventana |
| Filtering (FIR) | Filtro temporal de flujo |

Util como libreria matematica de apoyo, no para CV directo.

### ARM Arm-2D (`github.com/ARM-software/Arm-2D`)

- Biblioteca 2.5D para GUI en Cortex-M
- No relevante para vision por computadora
- No tiene kernels de procesamiento de imagen (blit, alpha-blending, transform — solo 2D rendering)

### ARM Arm NN (`github.com/ARM-software/armnn`)

- Motor de inferencia ML (TensorFlow Lite, ONNX)
- **Legacy**, ya no mantenido activamente
- Requiere Compute Library + demasiado pesado para OMAP3530 (Cortex-A8 600MHz)
- Si se quisiera ML ligero, mejor CMSIS-NN (pero para Cortex-M)

---

## Repos djtrance — Analisis Cruzado

| Repo | Relevancia | Por que |
|------|-----------|---------|
| **BeagleBoard-xM-video_encode_v4l2_rtp** | MUY ALTA | Mismo pipeline: V4L2 → DSP H.264 → RTP en SoC TI DM3730 (OMAP3530 successor). Referencia directa para validacion RTP. |
| **opencv-dsp-acceleration** | ALTA | Offloading de OpenCV a C64x+ DSP via DSPLink. Mismo DSP del OMAP3530. Enfoque reutilizable para acelerar pre-procesamiento en DSP. |
| **AR.Pwn** | ALTA | Acceso a camara AR.Drone en vuelo SIN matar program.elf. Elimina limitacion actual de requerir drone en tierra. |
| **ardrone1_video_decoder** | ALTA | Ecosistema AR.Drone — protocolo de video, navdata, RTP. Codigo de referencia. |
| **libOSD** | MEDIA-ALTA | Overlay YUV420/YUYV en C puro. Compatible directo con pipeline V4L2. Para incrustar telemetria en frame antes de H.264. |
| **boneCV** | MEDIA | Beaglebone Cortex-A8 + V4L2 + RTP. Patrones de streaming reutilizables. |
| **ezSIFT** | MEDIA | SIFT sin dependencias. Pesado para tiempo real a 600MHz sin DSP, pero portable. |
| **Curved-Lane-Lines** | BAJA | OpenCV desktop,车道 detection. No onboard. |
| **LineSLAM** | BAJA | ROS + OpenCV desktop. Demasiado pesado. |
| **fbg** | BAJA | 2D framebuffer para display. Drone no tiene display. |
| **ESP32-* (OticalFlow, CAM)** | **ALTA** | Los algoritmos son C puro y portables. Ver sección dedicada abajo. |

---

## Repos ESP32 — Algoritmos Portables a ARM

Los algoritmos de optical flow para ESP32 están escritos en C/C++ puro y se compilan sin cambios en ARM Cortex-A8. Solo el boilerplate (ESP-IDF, Arduino HAL) es específico de ESP32.

### 1. thomas-pegot/esp32-motion — Lucas-Kanade Denso + ARPS + EPZS

| Aspecto | Detalle |
|---------|---------|
| Algoritmos | LK denso (8b/16b), ARPS block matching, EPZS block matching, SAD cost function |
| Aritmética | Float (Gaussian 5×5, derivadas) + integer SAD (EPZS) |
| Resolución target | 96×96 @30fps, hasta 640×480 |
| Dependencias ESP32 | Solo `esp_heap_caps.h`, `esp_timer.h`, `esp_log.h` (wrappers de malloc y timer) |
| Dependencias del core | `math.h`, `stdlib.h`, `string.h`, `stdint.h` — **C89 puro** |
| Portabilidad | **Alta**. Reemplazar 2-3 calls: `heap_caps_aligned_calloc→malloc`, `esp_timer_get_time→clock_gettime`, `ESP_LOGE→printf` |
| Archivos clave | `lucas_kanade_opitcal_flow.c`, `block_matching.c`, `epzs.c` |

**Veredicto**: El candidato más sofisticado. LK denso configurable, con block matching como alternativa. Fácil de extraer.

### 2. PX4-OpticalFlow (forkeado a ESP32) — SAD Block Matching 8×8

| Aspecto | Detalle |
|---------|---------|
| Algoritmo | SAD block matching en tiles 8×8, búsqueda completa ±6px, subpixel por interpolación 8-directional SAD |
| Aritmética | **Entero puro** en loops centrales (uint8_t px, int accum). Float solo en subpixel y histograma |
| Resolución target | 64×64 a 128×160, output 15-1000 Hz |
| Dependencias | Originalmente STM32, fork ESP32 solo añade `esp_timer.h` |
| Portabilidad | **Muy alta**. Diseñado para Cortex-M4 (STM32F4), zero ESP32 en el algoritmo |
| Archivos clave | `px4flow.cpp`, `px4flow.hpp` |

**Veredicto**: El más ligero de todos. Probado en Cortex-M4. Pure integer. Ideal si queremos mínima carga de CPU.

### 3. qqqlab/ESP32-Optical-Flow — ARPS Block Matching

| Aspecto | Detalle |
|---------|---------|
| Algoritmo | Adaptive Rood Pattern Search (ARPS) con SAD cost function. Portado de MicroPython |
| Aritmética | **Entero puro** (int32_t, solo abs()). Sin math.h |
| Resolución target | 240×176 @~25fps |
| Dependencias del core | ~100 líneas de SAD + ARPS pattern search en C puro |
| Portabilidad | **Alta**. El algoritmo es una sola función. El resto es Arduino boilerplate |
| Archivo clave | `ESP32-Optical-Flow.ino` (la función central) |

**Veredicto**: El más simple. ARPS es más rápido que búsqueda completa pero menos preciso que LK.

### 4. OpenMV Phase Correlation — FFT + Log-Polar

| Aspecto | Detalle |
|---------|---------|
| Algoritmo | FFT phase correlation + log-polar transform (rotación/escala) |
| Aritmética | Float (fast_sqrtf, log, exp, complex multiply), tablas sin/cos precomputadas |
| Dependencias | `imlib.h`, `fft.h` — pure C. El FFT usa CMSIS-DSP (ARM-only) |
| Portabilidad | **Moderada**. Necesita backend FFT (NE10 o generic). El core phase correlation es portable |
| Archivo clave | `phasecorrelation.c` (679 líneas) |

**Veredicto**: Útil si se necesita estimación de rotación/escala. Heavy compute por la FFT. No es necesario para optical flow puro.

### Tabla Comparativa

| Proyecto | Algoritmo | Aritmética | Carga CPU | Portabilidad | Precisión |
|----------|-----------|-----------|-----------|-------------|-----------|
| esp32-motion | LK denso + ARPS + EPZS | Float + Integer | Media-Alta | Alta | **Alta** |
| PX4-OpticalFlow | SAD 8×8 tile | Entero puro | **Mínima** | **Muy alta** | Media |
| ESP32-Optical-Flow | ARPS | Entero puro | Baja | Alta | Baja-Media |
| OpenMV PhaseCorr | FFT phase correlation | Float | **Alta** | Moderada | Alta (rot+scale) |

### Corrección de mi análisis anterior

En la versión anterior de este documento clasifiqué los proyectos ESP32 como "BAJA relevancia, arquitectura diferente". **Esto fue incorrecto**. Los algoritmos son C/C++ puro, portables a cualquier arquitectura con un compilador de C. El Xtensa LX6 es irrelevante — el código fuente no usa instrucciones especializadas de ESP32.

Los únicos cambios necesarios para compilar en ARM Cortex-A8:
- `esp_timer_get_time()` → `clock_gettime(CLOCK_MONOTONIC, ...)`
- `heap_caps_aligned_calloc()` → `aligned_alloc()` o `memalign()`
- `ESP_LOGE()` → `fprintf(stderr, ...)` o eliminación
- `esp_camera.h` → nuestro propio V4L2 capture ya implementado

---

## Edge Flow UV vs Y — Bug Definitivamente Confirmado

### Conclusión: El código ACTUAL lee U/V por un ERROR, no por diseño

```diff
- sobel_sum += Sobel[c + 1] * (int32_t)img_buf[idx + 1];  // ORIGINAL: leía Y (correcto)
+ sobel_sum += Sobel[c + 1] * (int32_t)img_buf[idx];      // ACTUAL: lee U/V (bug)
```

**Commit `a75b2e8`** (TitusBraber, 23 Mar 2017): "corrected the index used for img_buf, the +1 caused a out of array memory address to be looked up"

**FALSO**: `idx+1` siempre está dentro del macropixel UYVY (4 bytes). El `+1` era intencional — skip del byte U/V para llegar a Y. El "fix" introdujo el bug.

### ¿Podría UV ser mejor que Y?

| Aspecto | Y (luma) | U/V (chroma) |
|---------|----------|--------------|
| Resolución horizontal | Full (1/px) | 2× menor (subsampling 4:2:2) |
| Frecuencia | Todas (incluye bordes reales) | Solo bajas (chroma suavizada) |
| Bordes por escena real | ~3-10× más comunes | Solo transiciones de color |
| Patrón por pixel | Y siempre en offset +1 | **Alterna U/V cada pixel** |

**El patrón alternante U/V es el problema crítico**:

```
x=1: gradiente = U₂ - U₀  (canal U)
x=2: gradiente = V₂ - V₀  (canal V) 
x=3: gradiente = U₄ - U₂  (canal U)
```

Columnas adyacentes miden canales diferentes y decorrelacionados. Un **desplazamiento de 1 pixel** intercambia U↔V, rompiendo la invarianza traslacional del histograma.

**En escena monocromática**: U/V ≈ 128 constante → histograma con CERO bordes → algoritmo produce basura.

### Veredicto

- **Teóricamente**: Y gana en casi todos los escenarios (mas bordes, sin alternancia, invarianza traslacional)
- **Único caso donde UV podría ayudar**: Muy baja luz, donde Y tiene mucho ruido y el lowpass inherente de UV podría filtrarlo. Pero la alternancia U/V anula esta ventaja
- **Corrección**: Volver a `img_buf[idx + 1]` o (mejor) convertir a grayscale antes, como ya hace LK

---

## AR.Pwn — Acceso a Cámara en Vuelo

### Mecanismo: LD_PRELOAD symbol interposition

AR.Pwn NO abre `/dev/video0` — secuestra la sesión V4L2 que ya tiene `program.elf`:

```
program.elf (con libhook.so precargada)
  ├── open("/dev/video0") → hook captura fd
  ├── ioctl(VIDIOC_QUERYBUF) → hook captura offsets DMA
  ├── mmap() → hook captura punteros a buffers
  └── ioctl(VIDIOC_DQBUF) → hook copia frame a /tmp/video0_buffer
       └── También soporta INYECCIÓN: si /tmp/video0_marked_ready existe,
           hook LEE el frame modificado y lo reescribe al buffer original
           antes de que program.elf lo procese
```

### Performance

| Métrica | Valor |
|---------|-------|
| Resolución hookeada | 640×480 (hardcodeado) |
| Framerate (sin -O3) | ~6.1 fps |
| Framerate (con -O3) | ~9.2 fps |
| Overhead por frame | write() de 460KB a tmpfs |
| Prioridad vision daemon | `setpriority(PRIO_PROCESS, 0, 19)` (mínima) |
| Procesamiento blob | 320×240 (chroma-sampled) |

### Ventajas vs. nuestro enfoque actual

| Aspecto | AR.Pwn | parrotFramework actual |
|---------|--------|----------------------|
| Convive con program.elf? | **Sí** (LD_PRELOAD) | No (killall program.elf) |
| Drone vuela? | **Sí** | No (pierde navdata) |
| Modificar frame inyectado? | **Sí** (marked_buffer) | No |
| Resolución 720p? | No (640×480 hardcodeado) | Sí |
| H.264 encoding? | Solo el de program.elf | Propio via DSP |

**Takeaway**: El mecanismo LD_PRELOAD es portable a nuestro framework. Podríamos crear un `libparrot_hook.so` que capture frames sin matar `program.elf` y los pase por shared memory (en vez de `/tmp`) a `drone_encoder`, permitiendo vuelo + captura + encode propio simultáneamente.

---

## Otros Algoritmos de Visión en Paparazzi

| Algoritmo | Archivo | Aritmética | Viabilidad OMAP | Uso potencial |
|-----------|---------|-----------|-----------------|--------------|
| **Color filter** | `colorfilter.c` | Entero (comparaciones) | **Excelente** | Detección de objetos por color |
| **Blob/CCL** | `blob_finder.c` | Entero (labels + merging) | **Buena** | Seguimiento de objetos color |
| **Snake gate** | `snake_gate_detection.c` | Entero+Float | **Buena** | Detección de aros/ventanas |
| **Window detection** | `detect_window.c` | Entero (integral image) | **Buena** | Detección bright-dark regions |
| **Size divergence** | `size_divergence.c` | Float (sqrt pairs) | **Buena** | Time-to-contact desde flow |
| **PnP+AHRS** | `PnP_AHRS.c` | Float (3×3 matrices) | **Buena** (N pequeño) | Localización 3D desde landmarks |
| **Geo-reference** | `cv_georeference.c` | **Fixed-point** puro | **Excelente** | Pixel→coordenadas NED |
| **QR code** | `qr_code.c` | ZBAR library | Moderada | Navegación por marcadores |
| **Textons** | `textons.c` | Float (3D arrays) | **Mala** (muy pesado) | Clasificación texturas |
| **Full undistort** | `undistort_image.c` | Float (trig/px) | **Mala** | Solo por corners, no full-frame |
| **IMAV marker** | `imavmarker.c` | Entero | Moderada | Patrones simétricos |

**No encontrado en Paparazzi**: SLAM, stereo, background subtraction, face detection, descriptor matching (SIFT/ORB), optical character recognition.

## Analisis Algoritmico Profundo

### FAST-9 Corner Detection (Paparazzi)

**Pipeline**:
```
fast9_detect(img, threshold, min_dist) →
  1. fast_make_offsets() — calcula offsets del circulo de Bresenham (16 pixeles, radio 3)
  2. Raster scan: para cada pixel (con skip por min_dist)
     └── Segment test: arbol de decision optimizado (Rosten)
           └── 9 de 16 pixeles contiguos deben ser todos brighter o darker que center±threshold
  3. min_dist suppression: interno, no usa non-max suppression con score
```

**Acceso a pixel**:
```c
const uint8_t *p = buf + y * img->w * ps + x * ps + ps/2;
p[pixel[i]];  // offset precalculado para pixel i del circulo
```

- `ps=2` para UYVY → `ps/2=1` → apunta al byte Y correctamente
- `row_stride = img->w` (tight packing asumido)
- **Ya funciona zero-copy sobre UYVY** — FAST extrae Y automaticamente

**Complejidad**: `O(W*H)` con ~4-6 comparaciones promedio por pixel.
**NEON**: Potencial 8-16x procesando 4 centros simultaneos, pero el arbol de decision optimizado tiene branches impredecibles.

**Parametros clave**: threshold (default 20, adaptativo [5,60]), min_dist (default 10), padding (default 20).

### Lucas-Kanade Optical Flow (Paparazzi)

**Pipeline** (Bouguet pyramidal LK):
```
Para cada punto, cada nivel de piramide:
  1. image_subpixel_window(old_img, window) → ventana W×W con interpolacion bilinear
  2. Gradientes espaciales: Ix = I(x+1)-I(x-1), Iy = I(y+1)-I(y-1)
  3. Tensor estructura: G = [ΣIx²/255  ΣIxIy/255; ΣIxIy/255  ΣIy²/255]
  4. Check: det(G) >= 1 (filtro de eigenvalue minimo)
  5. Iteraciones Newton-Raphson (max 5-20):
     a. window_J = subpixel_window(new_img) en posicion estimada
     b. diff = window_I - window_J
     c. b = [Σ(diff·Ix); Σ(diff·Iy)] / 255
     d. [vx; vy] = G⁻¹ · b
     e. flow += step → convergencia si |step_x|+|step_y| < threshold
```

**2×2 solve formula**:
```
G = [a  b]   G⁻¹ = 1/(a·c - b²) · [ c  -b]
    [b  c]                       [-b   a]

vx = (G[3]*b_x - G[1]*b_y) * subpixel_factor / Det
vy = (G[0]*b_y - G[2]*b_x) * subpixel_factor / Det
```

**Datos**: enteros puros (uint8_t px, int16_t grad, int32_t sumas, int64_t productos intermedios).
**Ventana**: box filter (sin peso gaussiano), `patch_size = 2*half_window+1`.
**Piramide**: kernel binomial [1,4,6,4,1]/16, edge reflection padding.
**Bug potencial**: overflow en `Det = G[0]*G[3] - G[1]*G[2]` si window > 16×16.

**NEON potential**: gradient computation 4-6×, tensor construction 4×, difference/multiply 4×.

### Edge Histogram Flow (Paparazzi)

**Pipeline**:
```
1. Edge histogram: para cada columna x, suma de |Gx| > threshold sobre todas las filas
                   para cada fila y, suma de |Gy| > threshold sobre todas las columnas
2. Derotation: shift histograma por diferencia de angulo del giroscopio
3. SAD block matching 1D: ventana 2W+1, search range [-D, +D]
4. Line fit: displacement[x] = divergence * x + flow (least-squares entero)
5. Velocidad: flow * fps * altitude / focal_length
```

**Kernel gradiente**: `[-1, 0, 1]` 1×3 (NO Sobel 3×3 completo — sin smoothing perpendicular).
**Bug critico**: En UYVY, `idx = interlace * (w*y + x)` lee byte 0 (U/V) en vez de byte 1 (Y). Edge histogram opera sobre crominancia, no luminancia. **Correccion**: `+1` offset para obtener Y.

**Complejidad**: `O(W*H + (W+H)*D*W)` — ~460× mas barato que LK denso.
**Memoria**: ~16KB para MAX_HORIZON=2.
**Fixed-point**: pipeline entero, floats solo en derotation y velocidad.

### CMSIS-CV — Kernels de Procesamiento

| Kernel | Coeficientes | Precision | NEON? |
|--------|-------------|-----------|-------|
| Sobel Gx | `[-1,0,1] * [1,2,1]` | Integer → Q15 | Solo Helium (Cortex-M) |
| Sobel Gy | `[1,2,1] * [-1,0,1]` | Integer → Q15 | Idem |
| Gaussian 3×3 | `[1,2,1]` separable, /16 | Q3→Q6→u8 | Idem |
| Gaussian 5×5 | `[1,4,6,4,1]` separable, /256 | Integer→u8 | Idem |
| Canny edge | Sobel + magnitud + NMS + hysteresis | Q15+atan2 | Idem |
| Resize | Bilinear interpolation | float32 | Idem |

**Constatacion critica**: Cero (`0`) `#ifdef ARM_MATH_NEON` en todo el codebase. Todo SIMD es Helium (MVE) exclusivo para Cortex-M55/M85. No compila en Cortex-A8.

**Perdida de oportunidad**: Los kernels separables (Sobel, Gaussian) mapean perfecto a NEON. Habria que escribirlos a mano.

### ezSIFT — Escalabilidad en OMAP3530

**Pipeline SIFT clasico**: 7 etapas, todas en float32.

| Etapa | Ops dominantes | NEON? | Tiempo estimado QVGA@600MHz |
|-------|---------------|-------|---------------------------|
| Gaussian pyramid (30 conv separables) | O(N * R) MACs | Alto (~60% runtime) | 100-200ms |
| DoD pyramid | O(N) subs | Trivial | 10-20ms |
| Gradient (mag + angle) | sqrt + atan2 por pixel | Moderado | 30-50ms |
| Keypoint detection | 26-neighbor + 3×3 Hessian solve | Bajo (branches) | 50-200ms |
| Orientation | exp + histogram | Bajo (scatter) | 30-100ms |
| Descriptor | trilinear interp + exp | Bajo (scatter) | 50-200ms |
| **Total** | | | **~300ms - 1s** |

**Memoria pico** (float32):
- QVGA (320×240): ~7 MB
- 720p (1280×720): ~60 MB (de ~128MB totales en OMAP3530 — factible pero apretado)

**Bottom line**: SIFT completo en QVGA a ~1-3 fps en Cortex-A8 600MHz con NEON. No es tiempo real para 30fps pero util para inicializacion de SLAM o mapeo de baja frecuencia.

## Estrategia Y-Channel-Only para Optical Flow

### Enfoque 1: Zero-copy sobre UYVY (sin extraer Y)

Modificar los algoritmos de Paparazzi para que acepten un stride explicito en lugar de asumir tight packing:

```c
// En lugar de:
pixel_at(x,y) = buf[y * w * ps + x * ps + ps/2];

// Usar stride-aware:
pixel_at(x,y) = buf[y * stride + x * ps + ps/2];
// stride = bytesperline de V4L2
// ps = 2 para UYVY
```

- **FAST**: Modificar `fast_make_offsets()` para usar `stride` en vez de `w * ps`
- **LK**: Requiere ventanas de subpixel → necesita grayscale contiguo → inevitable copia
- **Edge Flow**: Corregir offset de Y (+1) y usar stride

### Enfoque 2: NEON Y-extraction (una copia, rapida)

Usar NEON `vld4.8` para deinterleaves UYVY a planar Y:

```c
// NEON: carga 16 bytes UYVY, extrae Y samples
uint8x8x4_t uyvy = vld4_u8(frame_ptr);
// uyvy.val[0] = U, val[1] = Y, val[2] = V, val[3] = Y
// Extraer Y:
uint8x8_t y_even = uyvy.val[1];  // Y0, Y2, Y4...
uint8x8_t y_odd  = uyvy.val[3];  // Y1, Y3, Y5...
```

- ~4× mas rapido que copia byte a byte
- Produce buffer grayscale contiguo para LK y Edge Flow
- 1 copia vs 1.5 copias de NV12 conversion (ganancia del 33%)

### Enfoque 3: In-place compaction (destructivo, zero alloc)

Como no necesitamos U/V para optical flow, podemos compactar el buffer UYVY in-place moviendo los bytes Y a la mitad frontal:

```
Antes:  U0 Y0 V0 Y1 U2 Y2 V2 Y3 ...
Despues: Y0 Y1 Y2 Y3 ... (primeros width*height bytes)
```

- No requiere memoria adicional
- Destructivo (pierde U/V)
- Util si solo se necesita Y

### Enfoque 4: Pipeline Optimo Propuesto

```
V4L2 UYVY frame (4 buffers en anillo)
  │
  ├──→ [para H.264 encode] uyvy_to_nv12() + memcpy a DSP buf
  │
  └──→ [para optical flow] puntero Y via stride-aware access
       └──→ FAST corners (directo sobre UYVY, zero-copy)
       └──→ LK tracking (NEON Y-extract → grayscale temporal)
       └──→ Resultado: flow_x, flow_y, divergence

  Y channel extraido via NEON vld4 → width*height bytes (opcional si LK se usa)
  Corner detection: zero-copy sobre UYVY (FAST ya soporta)
```

### Mapa de Ancho de Banda por Frame

| Operacion | Bytes copiados | Penalizacion CPU |
|-----------|---------------|------------------|
| `uyvy_to_nv12()` actual | `w*h*1.5` | Alta (C loop) |
| NV12 → DSP `memcpy` | `w*h*1.5` | Media (DMA-like) |
| **Total encode pipeline** | **`w*h*3.0`** | |
| Y extraction (NEON vld4) | `w*h*1.0` | Baja (NEON) |
| Y pointer + FAST (zero-copy) | 0 | Minima (solo puntero) |
| LK con grayscale temporal | `w*h*1.0` (Y extraction) | Media |

**Bottom line**: Y-channel-only para optical flow elimina `w*h*1.5` de copia vs el pipeline actual con NV12. Adicionalmente, FAST corners funciona con zero-copy directo sobre UYVY.

---

---

## Matriz Completa de Capacidades CV para OMAP3530

### Categorias de Funcionalidades

Cada funcionalidad tiene un puntaje de factibilidad:

| Puntaje | Significado |
|---------|-------------|
| ★★★★★ | Listo para portar (codigo C existe, bajo costo) |
| ★★★★☆ | Portable con modificaciones menores |
| ★★★☆☆ | Posible pero requiere desarrollo significativo |
| ★★☆☆☆ | Muy dificil en OMAP3530 (muy pesado) |
| ★☆☆☆☆ | Impracticable en este hardware |

---

### 1. ESTIMACION DE MOVIMIENTO (Motion Estimation)

| # | Funcionalidad | Algoritmo | Fuente | LOC | Aritmetica | Resolucion | Memoria | CPU est. 600MHz | Fact. |
|---|--------------|-----------|--------|-----|-----------|-----------|---------|-----------------|-------|
| 1.1 | **Optical Flow horizontal (vx, vy)** | LK + FAST9 sparse | Paparazzi | ~500 | int32 + float | QVGA | ~2 MB | ~5-15ms | ★★★★★ |
| 1.2 | **Optical Flow horizontal (vx, vy)** | EdgeFlow histogram | Paparazzi | ~300 | int32 puro | QVGA | ~16 KB | ~2-5ms | ★★★★★ |
| 1.3 | **Optical Flow horizontal (vx, vy)** | SAD block matching 8×8 | PX4-OpticalFlow | ~400 | entero puro | 64×64 | ~64 KB | ~1-3ms | ★★★★★ |
| 1.4 | **Optical Flow denso** | Lucas-Kanade denso | esp32-motion | ~350 | float | 96×96 | ~1 MB | ~10-30ms | ★★★★☆ |
| 1.5 | **Optical Flow horizontal (vx, vy)** | ARPS block matching | qqqlab/ESP32-OF | ~100 | int32 puro | 240×176 | ~128 KB | ~2-8ms | ★★★★★ |
| 1.6 | **Divergence (expansion/contraccion)** | Size divergence (pairs) | Paparazzi `size_divergence.c` | ~95 | float (sqrtf) | QVGA+ | ~64 KB | ~0.5-5ms | ★★★★★ |
| 1.7 | **Divergence (expansion/contraccion)** | Linear flow fit (RANSAC+SVD) | Paparazzi `linear_flow_fit.c` | ~300 | float (SVD) | QVGA+ | ~2 MB | ~5-20ms | ★★★★☆ |
| 1.8 | **Focus of Expansion (FoE)** | Linear flow fit → intersection | Paparazzi `linear_flow_fit.c` | ~10 | float | QVGA+ | ~2 MB | ~5-20ms | ★★★★☆ |
| 1.9 | **Rotacion visual (w/ gyro fusion)** | Derotation por gyro | Paparazzi `opticflow_calculator.c` | ~80 | float + trig | Cualquiera | ~1 KB | ~0.1ms | ★★★★★ |
| 1.10 | **Rotacion visual (pura, sin gyro)** | Alkowatly 6-param flow fit | Paparazzi (comentado) | ~80 | float + SVD | QVGA+ | ~2 MB | ~5-20ms | ★★★☆☆ |
| 1.11 | **Rotacion + escala (phase corr)** | FFT log-polar | OpenMV `phasecorrelation.c` | ~679 | float + FFT | 64×64 | ~8 MB | ~20-50ms | ★★☆☆☆ |
| 1.12 | **Velocidad vertical (vz)** | divergence * altitude | Paparazzi `opticflow_calculator.c` | ~5 | float | - | - | ~0.01ms | ★★★★★ |

#### Detalle 1.1: LK + FAST9 Sparse

**Pipeline completo (opticflow_calculator.c)**:
```
image_to_grayscale() → FAST9 corners → pyramid build →
LK tracking → median filter → derotation → velocity calc
```

**Configuraciones clave**: 25 corners, window 10px, search 20px, 2 pyramid levels, 10 iteraciones.
**Pixel access**: Convierte UYVY → grayscale primero (`image_to_grayscale`). Despues acceso `uint8_t*` directo.
**Memoria extra**: pyrámide = ~1.3× imagen grayscale = ~307KB para QVGA.
**Loopy parts**:
- FAST9: ~4-6 cmp/px, O(W×H)
- LK por corner: O(W×W×I×N) donde W=window, I=iteraciones, N=corners
- Random sample divergence: O(n_samples)

**Known issues**: 
- `image_to_grayscale()` en image.c: `source++` skip U/V actual, extrae Y correctamente ✅
- `image_gradients()` usa kernel `[0 -1 0; -1 0 1; 0 1 0]` (NO Sobel clasico de 3×3)
- Overflow potencial en `image_calculate_g()`: `sum_dxx += dx*dx` puede desbordar int32 con window grande (>181px)

#### Detalle 1.2: EdgeFlow

**Pipeline completo (edge_flow.c + opticflow_calculator.c)**:
```
calculate_edge_histogram('x') → calculate_edge_histogram('y') →
calculate_edge_displacement(x) → calculate_edge_displacement(y) →
line_fit(x) → line_fit(y) → derotation → velocity calc
```

**BUG CONFIRMADO edge_flow.c:104**: `sobel_sum += Sobel[c + 1] * (int32_t)img_buf[idx]` donde `idx = interlace * (image_width * y + (x + c))`. Para UYVY (interlace=2), lee el byte U/V en offset 0. **Correccion**: El codigo ORIGINAL tenia `img_buf[idx + 1]` que lee Y. El commit `a75b2e8` "corrigio" cambiando `idx+1` a `idx`, introduciendo el bug.

**Memory**: Solo 2 histogramas de tamaño W y H. MAX_HORIZON=2 frames guardados→ ~16KB para 720p.

**Complexity**: O(W×H + (W+H)×D×W) donde D=disp_range, W=window_size.
~460× mas barato que LK denso.

**Ventaja clave**: No requiere grayscale. Opera directamente sobre UYVY (una vez corregido el bug).

---

### 2. DETECCION DE OBJETOS

| # | Funcionalidad | Algoritmo | Fuente | LOC | Aritmetica | Resolucion | Memoria | CPU est. 600MHz | Fact. |
|---|--------------|-----------|--------|-----|-----------|-----------|---------|-----------------|-------|
| 2.1 | **Filtro de color YUV** | Threshold 6-ejes | Paparazzi `colorfilter.c` + `image.c` | ~70 | uint8_t cmp | Full HD | ~1 KB | ~1-3ms@QVGA | ★★★★★ |
| 2.2 | **Blob detection (CCL)** | Connected Component Labeling | Paparazzi `blob_finder.c` | ~200 | entero | QVGA+ | ~W×H labels | ~3-10ms | ★★★★☆ |
| 2.3 | **Snake Gate / Ring detection** | Snaking + outline check | Paparazzi `snake_gate_detection.c` | ~1100 | entero + float | QVGA | ~1 KB | ~5-20ms | ★★★★☆ |
| 2.4 | **Window detection** | Integral image + sliding window | Paparazzi `detect_window.c` | ~350 | entero puro | QVGA | ~W×H int32 | ~10-50ms | ★★★★☆ |
| 2.5 | **Deteccion de circulos** | Snake gate adaptado + ellipse | - | - | - | - | - | - | ★★★☆☆ |
| 2.6 | **Deteccion de personas** | HOG / LBP cascade | OpenCV (no portable) | - | - | - | - | - | ★☆☆☆☆ |
| 2.7 | **QR Code** | ZBAR library | Paparazzi `qr_code.c` | ~500 | entero | VGA+ | ~2 MB | ~20-100ms | ★★★☆☆ |
| 2.8 | **AprilTag / ArUco** | AprilTag 3 C library | AprilTag standalone | ~2000 | float | QVGA+ | ~4 MB | ~10-50ms | ★★★☆☆ |
| 2.9 | **IMAV markers** | Pattern matching | Paparazzi `imavmarker.c` | ~300 | entero | VGA | ~1 MB | ~10-30ms | ★★★★☆ |
| 2.10 | **Path / line detection** | Edge histogram + line_fit() | Paparazzi `edge_flow.c` | ~50 | int32 | Cualquiera | ~16 KB | ~2-5ms | ★★★★★ |
| 2.11 | **Orange avoider (obstaculo color)** | state machine + color count | Paparazzi `orange_avoider.c` | ~200 | uint8_t cmp | Cualquiera | ~1 KB | ~1ms | ★★★★★ |

#### Detalle 2.1: Color Filter

**Algoritmo** (`image.c` `image_yuv422_colorfilt()`):
```c
for each pixel pair UYVY:
  if(Y >= y_m && Y <= y_M && U >= u_m && U <= u_M && V >= v_m && V <= v_M)
    cnt++; dest[0]=64; dest[1]=Y; dest[2]=255; dest[3]=Y;  // highlight
  else
    dest[0]=U_source; dest[1]=Y; dest[2]=V_source; dest[3]=Y;  // dim
```

**Zero-copy**: Opera IN-PLACE sobre el buffer UYVY. Sin copia.
**Pixel access**: Recorre UYVY, lee de `source`, escribe a `dest` (puede ser mismo buffer).
**Color check per-pixel**: `check_color_yuv422()` maneja odd/even pixel alineacion.

**Para deteccion de cielo**: Threshold azul cielo tipico: Y=[100,200], U=[90,140], V=[140,200] (U es Cb, V es Cr). 8-bit puro, streaming, ~33-50% de la CPU de FAST corners por frame.

#### Detalle 2.2: Blob Finder

Basado en connected component labeling. Loopea sobre imagen filtrada por color y asigna labels. Union-Find con path compression. Despues extrae centroide, area, bounding box de cada blob. Entero puro.

#### Detalle 2.3: Snake Gate Detection

**Algoritmo complejo**:
1. `n_total_samples` puntos aleatorios en la imagen
2. Por cada muestra del color objetivo:
   a. `snake_up_and_down()` → encuentra extremos verticales del segmento de color
   b. `snake_left_and_right()` → encuentra extremos horizontales (x2: top y bottom)
   c. Construye estimacion de 4 esquinas del gate
3. `check_gate_outline()` verifica que los 4 lados tengan ≥40% pixeles del color correcto
4. `check_inside()` verifica que el interior NO sea del color objetivo
5. `gate_refine_corners()` histograma local para refinar posiciones de esquinas
6. Non-max suppression via IoU threshold (0.7)
7. Memoria temporal entre frames para tracking

**Pixel access**: `check_color_snake_gate_detection()` → `check_color_yuv422()` en image.c
**Aritmetica**: Mayormente entero. Float solo en:
- `check_line()`: interpolacion t_step=0.05
- `segment_length()`: sqrtf
- `intersection_over_union()`: float division
- `refine_single_corner()`: float boundary calculations

**Performance**: El numero de muestras (n_samples) controla directamente el tiempo de computo. 100 muestras → ~5ms en Bebop (Cortex-A9). En OMAP3530: ~10-20ms. Las funciones de snaking son O(k) donde k es el tamaño del gate en pixeles.

#### Detalle 2.4: Window Detection

**Algoritmo**: Integral image + sliding window de brillo/oscuridad. 
- Calcula integral image (O(W×H))
- Para cada posicion (x,y) y tamano (s): computa response = bright_outer / dark_inner
- Busca ventanas donde el interior es mas oscuro (o mas brillante) que el borde

**Pixel access**: Opera sobre grayscale (convierte via `image_to_grayscale()` primero)
**Aritmetica**: Entero puro uint32_t. Sin float, sin sqrt, sin trig.
**Resolution**: Ideal para VGA (~2MB integral image). QVGA (~307KB).
**Complexity**: O(W×H + N_windows). Con ventana 100px stride 1: ~100K windows. Cada window: 4 accesos a integral image.

---

### 3. NAVEGACION

| # | Funcionalidad | Algoritmo | Fuente | LOC | Aritmetica | Resolucion | Memoria | CPU est. 600MHz | Fact. |
|---|--------------|-----------|--------|-----|-----------|-----------|---------|-----------------|-------|
| 3.1 | **Odometria visual (vx, vy, vz)** | OF + divergence + altitude | Paparazzi integrated | ~800 | mixto | QVGA | ~2 MB | ~10-30ms | ★★★★★ |
| 3.2 | **Estimacion de pendiente (slope)** | Linear flow fit params | Paparazzi `linear_flow_fit.c` | ~60 | float | QVGA | ~2 MB | ~5-20ms | ★★★★☆ |
| 3.3 | **Time-to-Contact (TTC)** | 1/divergence | Paparazzi `size_divergence.c` | ~95 | float (sqrtf) | QVGA+ | ~64 KB | ~0.5-5ms | ★★★★★ |
| 3.4 | **Colision avoidance (monocular)** | TTC + FoE + steering | Derivado de Paparazzi | ~200 | mixto | QVGA | ~2 MB | ~10-30ms | ★★★★☆ |
| 3.5 | **Colision avoidance (stereo)** | Disparity → potential fields | Paparazzi `obstacle_avoidance.c` | ~700 | float | - | - | - | ★☆☆☆☆ |
| 3.6 | **Visual landing** | Divergence → descenso | Paparazzi integrated | ~50 | float | QVGA | ~64 KB | ~1ms | ★★★★★ |
| 3.7 | **Visual return-to-home (snapshot)** | Min-warping / image matching | Academic (no C code) | ~300 | float + FFT | QVGA | ~4 MB | ~20-100ms | ★★☆☆☆ |
| 3.8 | **Visual return-to-home (dead reckoning)** | OF integration + IMU fusion | Derivado | ~150 | float | - | ~1 KB | ~0.1ms | ★★★★★ |
| 3.9 | **Path/corridor following** | Edge histogram + line_fit | Paparazzi `edge_flow.c` | ~50 | int32 | Cualquiera | ~16 KB | ~2-5ms | ★★★★★ |
| 3.10 | **Geo-referencing** | Fixed-point projection | Paparazzi `cv_georeference.c` | ~120 | **fixed-point puro** | Cualquiera | ~1 KB | ~0.01ms | ★★★★★ |
| 3.11 | **PnP localization (3D desde 2D-3D)** | Perspective-n-Point | Paparazzi `PnP_AHRS.c` | ~300 | float (SVD) | QVGA+ | ~4 MB | ~5-20ms | ★★★★☆ |
| 3.12 | **Waypoint navigation visual** | PnP + AHRS + geo-ref | Paparazzi integrated | ~500 | mixto | QVGA | ~4 MB | ~10-50ms | ★★★★☆ |

#### Detalle 3.4: Collision Avoidance Monocular

**Algoritmo propuesto** (NO existe como modulo en Paparazzi, pero se deriva directamente):
```
1. Calcular optical flow (LK sparse o EdgeFlow)
2. Extraer FoE y divergence (linear_flow_fit)
3. Si divergence > threshold → obstacle approaching
4. Direccion de escape = opuesta al FoE
5. Steering command: ref_roll = k * (FoE_x - center_x)
```

**Diferencia critica con stereo**: El modulo `obstacle_avoidance.c` de Paparazzi requiere disparidad stereo (2 camaras) o sensor de profundidad. AR.Drone 2.0 tiene SOLO monocular frontal. La unica manera de detectar obstaculos es:
1. **Por expansion (TTC)**: Un obstaculo frontal causa divergencia positiva alta en el flow
2. **Por ausencia de textura**: Superficie uniforme (pared blanca) → pocos corners → perdida de tracking
3. **Por color**: Si el obstaculo tiene un color especifico (ej: naranja competencia)

**Viabilidad**: ★★★★☆ para divergencia-based. ★★★★★ para color-based (via colorfilter).

#### Detalle 3.7: Visual Return-to-Home (Snapshot)

**Estado actual**: NO existe implementacion en C puro open-source para drones. Existe:
- **Bee-Nav** (TU Delft, Nature 2026): Min-warping visual homing, C++/ROS, no portable directamente
- **ALV / Average Landmark Vector**: Bee-inspired, requiere tracking de landmarks visuales
- **Snapshot matching**: Guardar imagen de despegue, correlacionar en retorno

**Implementacion necesaria**:
```c
// Approach: Guardar descripcion compacta del punto de partida
// 1. Al despegar: extraer FAST corners + BRIEF descriptors (o similar)
// 2. Durante vuelo: dead reckoning con OF + IMU
// 3. En retorno: matching de descriptores actuales vs guardados
// 4. Correccion de deriva cuando hay match
```

**Alternativa mas practica**: Dead reckoning puro (OF + IMU fusion). El error de deriva es inevitable pero para retornos cortos (<100m) funciona.
**No requiere codigo nuevo**: Solo integrar los modulos existentes de OF + IMU.

---

### 4. ENTORNO / ESCENA

| # | Funcionalidad | Algoritmo | Fuente | LOC | Aritmetica | Resolucion | Memoria | CPU est. 600MHz | Fact. |
|---|--------------|-----------|--------|-----|-----------|-----------|---------|-----------------|-------|
| 4.1 | **Sky detection** | YUV color threshold + edge analysis | Derivado de colorfilter + edge_flow | ~80 | uint8_t cmp + int32 | Cualquiera | ~1 KB | ~1-3ms | ★★★★★ |
| 4.2 | **Ground plane estimation** | Linear flow fit (slope_x, slope_y) | Paparazzi `linear_flow_fit.c` | ~30 | float | QVGA | ~2 MB | ~5-20ms | ★★★★☆ |
| 4.3 | **Surface roughness** | Linear flow fit error | Paparazzi `linear_flow_fit.c` | ~5 | float | QVGA | ~2 MB | ~5-20ms | ★★★★☆ |
| 4.4 | **Texture analysis** | Texton dictionary | Paparazzi `textons.c` | ~400 | float (log2, malloc) | QVGA | ~8 MB | ~20-100ms | ★★☆☆☆ |
| 4.5 | **Horizon detection** | Max edge row in histogram | Derivado de edge_flow | ~20 | int32 | Cualquiera | ~16 KB | ~2-5ms | ★★★★★ |
| 4.6 | **Undistortion** | By-corner remap | Paparazzi `undistortion.c` | ~200 | float (trig/px) | Solo corners | ~1 KB | ~0.1ms/corner | ★★★★☆ |

#### Detalle 4.1: Sky Detection

**No existe en Paparazzi**. Se puede implementar combinando:
1. **Color filter**: Azul de cielo ≈ Y=[100,200], U=[90,150], V=[130,200]
2. **Edge histogram horizontal**: La fila con maximo gradiente horizontal = horizonte (linea de transicion cielo/tierra)
3. **Resultado**: Porcentaje de cielo en la mitad superior del frame.

```c
// Algoritmo propuesto (~80 LOC):
// 1. sky_pixels = image_yuv422_colorfilt con threshold azul
// 2. edge_histogram_y = filas con >30% azul → cielo
// 3. horizon_row = max edge en edge_histogram_y
// 4. sky_ratio = pixels azules / total pixels
```

---

### 5. FUNCIONALIDADES AVANZADAS

| # | Funcionalidad | Algoritmo | Fuente | LOC | Aritmetica | Resolucion | Memoria | CPU est. 600MHz | Fact. |
|---|--------------|-----------|--------|-----|-----------|-----------|---------|-----------------|-------|
| 5.1 | **SIFT / ORB features** | Gaussian pyr + DoD + descriptor | ezSIFT | ~2000 | float (trig, exp) | QVGA | ~7 MB | ~300-1000ms | ★★☆☆☆ |
| 5.2 | **Background subtraction** | Frame differencing + threshold | Sencillo | ~30 | uint8_t cmp | Cualquiera | ~1 MB | ~1-3ms | ★★★★★ |
| 5.3 | **Motion detection** | Frame diff + bounding box | Sencillo | ~50 | uint8_t | Cualquiera | ~1 MB | ~1-5ms | ★★★★★ |
| 5.4 | **Panorama stitching** | Feature matching + warp | OpenCV (no portable) | - | float | - | - | - | ★☆☆☆☆ |
| 5.5 | **Structure from Motion** | Multi-view triangulation | Academic | - | float | - | - | - | ★☆☆☆☆ |
| 5.6 | **Visual SLAM** | EKF / Graph SLAM | Academic | - | float | - | - | - | ★★☆☆☆ |

#### Detalle 5.1: SIFT

**Pipeline**: Gaussian pyramid (30 conv) → DoD → Keypoint detection → Orientation → Descriptor.
**Tiempo estimado**: ~300ms-1s en QVGA. Solo util para inicializacion o mapping offline.
**No es tiempo real** para control de vuelo. Pero puede ser util para:
- Capturar snapshot de despegue (una vez, al inicio)
- Matching de landmarks visuales para correccion de deriva
- Mapeo de baja frecuencia (~1 cada 5 segundos)

---

### 6. RESUMEN: CAPACIDADES POR PRIORIDAD DE IMPLEMENTACION

#### Fase 1: Inmediato (codigo existe listo para portar)

| Capacidad | Archivos | LOC total | Dependencias |
|-----------|---------|-----------|-------------|
| **FAST9 corners** | `fast_rosten.c` | ~200 | image.h structs |
| **EdgeFlow** | `edge_flow.c` + `opticflow_calculator.c` | ~400 | image.h + correccion bug UYVY |
| **Size divergence** | `size_divergence.c` | ~95 | flow_t struct |
| **Color filter** | `colorfilter.c` + `image.c` (yuv422_colorfilt) | ~70 | image.h |
| **Image utilities** | `image.c` + `image.h` | ~500 | stdlib |
| **Geo-referencing** | `cv_georeference.c` | ~120 | estado drone |
| **Total Fase 1** | | **~1385 LOC** | |

**Tiempo estimado de port**: ~2-4 horas (adaptar structs, eliminar dependencias Paparazzi).

#### Fase 2: Corto plazo (modificaciones menores necesarias)

| Capacidad | Modificacion necesaria | LOC | Complejidad |
|-----------|----------------------|-----|-------------|
| **LK tracking** | Extraer Y via NEON vld4 (o image_to_grayscale) | ~350 | Media |
| **Linear flow fit** | Reemplazar SVD de Paparazzi + RANSAC | ~300 | Alta (matematica) |
| **Snake gate** | Adaptar coordinate system, eliminar Bebop hacks | ~1100 | Media |
| **Window detection** | Convierte a grayscale primero (ya existe) | ~350 | Baja |
| **Orange avoider** | State machine standalone, sin ABI | ~200 | Baja |
| **Total Fase 2** | | **~2300 LOC** | |

#### Fase 3: Mediano plazo (desarrollo necesario)

| Capacidad | Que se necesita | LOC est. |
|-----------|----------------|---------|
| **FoE + collision avoidance** | Integrar linear_flow_fit + steering logic | ~200 |
| **Sky detection** | colorfilter + edge_histogram combinacion | ~80 |
| **Horizon detection** | Derivado de edge_histogram horizontal | ~30 |
| **Visual landing** | divergence threshold + descenso control | ~100 |
| **PnP localization** | Portar logica PnP (no encontre el archivo) | ~300 |
| **Path/corridor following** | edge_flow line_fit adaptado | ~100 |
| **Total Fase 3** | | **~810 LOC** |

#### Fase 4: Largo plazo (investigacion + desarrollo)

| Capacidad | Que se necesita | LOC est. | Riesgo |
|-----------|----------------|---------|--------|
| **Visual return (snapshot)** | Min-warping implementation | ~300 | Alto (investigacion) |
| **Sparse SIFT** | Portar ezSIFT, solo para init | ~500 | Medio |
| **AprilTag** | Portar AprilTag 3 C library | ~2000 | Bajo (codigo existe) |
| **Visual SLAM** | Integrar ORB + EKF | ~1500 | Alto |

---

### 7. RESTRICCIONES DEL SISTEMA COMPLETO

**No todas las capacidades pueden ejecutarse simultaneamente.** Presupuesto de CPU:

| Proceso | CPU estimada @720p | CPU estimada @QVGA |
|---------|-------------------|-------------------|
| V4L2 capture | ~5% | ~2% |
| uyvy_to_nv12() | ~15% | ~3% |
| H.264 encode (DSP offload) | ~0% CPU | ~0% CPU |
| **Subtotal encode** | **~20%** | **~5%** |
| FAST9 corners | ~10% | ~3% |
| LK tracking (25 pts) | ~15% | ~5% |
| EdgeFlow | ~5% | ~2% |
| Color filter | ~8% | ~2% |
| Snake gate (100 samples) | ~20% | ~10% |
| **Subtotal CV** | **~15-40%** | **~5-15%** |
| Navdata + AT commands | ~2% | ~2% |
| Control loops | ~5% | ~5% |
| **Total typical** | **~40-65%** | **~15-30%** |

**Conclusión**: A 720p, se puede correr H.264 encode + 1 algoritmo CV (EdgeFlow o colorfilter) con ~40% CPU libre. A QVGA, podemos correr encode + 2-3 algoritmos CV simultaneamente.

**Para vuelo autonomo**: Usar QVGA para CV (downsample via `image_yuv422_downsample()`), 720p solo para H.264 encode. Esto deja ~70% CPU para algoritmos de navegacion.

---

## Resumen Ejecutivo

Del analisis de ~9000 LOC en total de Paparazzi + PX4 + ESP32 + OpenMV, estas son las capacidades REALMENTE portables al OMAP3530:

| Categoria | ★★★★★ | ★★★★☆ | ★★★☆☆ | ★★☆☆☆ | ★☆☆☆☆ |
|-----------|-------|--------|--------|--------|--------|
| Motion estimation | 6 | 3 | 1 | 1 | 0 |
| Object detection | 4 | 3 | 2 | 0 | 1 |
| Navigation | 6 | 4 | 0 | 2 | 1 |
| Environment | 2 | 3 | 0 | 1 | 0 |
| Advanced | 2 | 0 | 0 | 2 | 2 |

**Total factible (★★★★★ + ★★★★☆)**: **30 capacidades**
**Total NO factible (★★☆☆☆ + ★☆☆☆☆)**: **7 capacidades**
**Total marginal (★★★☆☆)**: **3 capacidades**

---

## Recomendaciones Finales

1. **FAST corners**: Usar directamente sobre UYVY (Paparazzi ya lo soporta, solo asegurar stride correcto)
2. **Lucas-Kanade**: Extraer Y via NEON `vld4` → buffer grayscale temporal (no reusar NV12 buf)
3. **Edge Flow**: Corregir para leer Y (+1 offset) en vez de U/V (+0 offset)
4. **CMSIS-CV**: No incluir como dependencia. Los Sobel/Gaussian son ~100 LOC cada uno, mejor implementar manualmente con NEON si se necesita rendimiento
5. **CMSIS-DSP**: Solo la parte de matrix 2×2 solve para LK (~50 LOC), no vale la pena toda la libreria
6. **DSP offloading** (via opencv-dsp-acceleration): Evaluar si el costo de DSPLink communication vale la pena vs correr en NEON a 600MHz
7. **Pipeline recomendado para vuelo autonomo**: QVGA (downsample 4× via `image_yuv422_downsample()`) → EdgeFlow (global flow) + FAST9 (features sparse) + colorfilter (sky) + divergence (TTC). Todo en ~15% CPU. H.264 encode a 720p en DSP simultaneamente.
8. **Orden de implementacion**: Fase 1 (infraestructura imagen) → Fase 2 (core flow) → Fase 3 (navegacion) → Fase 4 (avanzado)
