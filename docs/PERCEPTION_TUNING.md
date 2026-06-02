# Percepción y calibración - flujo actual

Esta guía resume el stack que quedó para calibrar, probar y ajustar visión en el
Puzzlebot. La idea general es: cámara CSI directa por GStreamer, preview rápido
por H264, y una sola fuente de verdad para geometría, iluminación y detección de
intersecciones.

---

## Orden recomendado

1. **Enfoque del lente**

```bash
scripts/run_focus_assist_jetson.sh
```

Apunta a un objetivo con textura a la distancia de trabajo y gira el lente hasta
maximizar el valor de foco. Haz esto antes de calibrar y no vuelvas a tocar el
foco después.

2. **Intrínsecos con checkerboard**

```bash
scripts/run_checkerboard_capture_jetson.sh
scripts/run_calibrate_camera_jetson.sh
```

Resultado validado en esta sesión:

```text
30/30 imágenes detectadas
RMS reprojection error: 0.3449 px
Veredicto: EXCELENTE
pattern: 5x7
image_size: 640x480
```

Archivo generado y versionable:

```text
config/camera_params.npz
```

3. **Iluminación flat-field**

```bash
scripts/run_illumination_calibrator_jetson.sh
```

Ahora espera a que tú presiones `Enter`: acomodas la lona blanca viendo el H264,
presionas `Enter`, hay una pausa para retirar manos/sombras, y luego junta los
frames buenos automáticamente.

Resultado validado en esta sesión:

```text
std BGR antes : 3.4 4.4 14.5
std BGR despues: 0.6 0.6 0.8
```

Archivo generado y versionable:

```text
config/illumination_flatfield.npz
```

4. **Probar Otsu, máscara y ROIs**

```bash
scripts/run_line_calibrator_jetson.sh
```

Por defecto abre un dashboard H264 rápido con:

- vista procesada con overlay
- máscara Otsu
- panel de debug compacto

Para volver al modo viejo con trackbars OpenCV:

```bash
STREAM=local scripts/run_line_calibrator_jetson.sh
```

---

## Cómo probar la cámara sola

Cámara cruda sin calibraciones:

```bash
scripts/run_camera_h264_jetson.sh
```

Preview con calibraciones aplicadas, sin mover el robot:

```bash
scripts/run_recorder_jetson.sh
```

Si no presionas `Enter`, se queda como preview. Este camino carga
`camera_params.npz` y `illumination_flatfield.npz`.

---

## Otsu y máscara negra

La máscara de línea/intersección vive en:

```text
puzzlebot_ros/perception/intersection.py
```

La función clave es `black_mask(frame)`:

```text
BGR -> grayscale -> Gaussian blur -> Otsu inverse threshold -> morphology open
```

En el dashboard H264, la mitad derecha muestra esa máscara. Lo esperado:

- línea negra y cuadritos negros de la pista aparecen blancos en la máscara
- fondo claro aparece negro
- sombras, silla, cables y fondo del laboratorio no deberían dominar la máscara

Si hay mucho ruido, ajusta filtros del detector antes de tocar el algoritmo de
Otsu.

---

## Dashboard H264 del line calibrator

Comando principal:

```bash
scripts/run_line_calibrator_jetson.sh
```

En H264 no hay sliders reales porque H264 solo transmite video. Los cambios se
hacen por terminal o desde otro shell con `set_calibrator_param.sh`.

Comandos dentro de la terminal del calibrador:

```text
min_dash_count=6
roi_y0_pct=72
roi_skew=8
s=1
p=1
u=1
q=1
```

Comandos desde otra terminal:

```bash
scripts/set_calibrator_param.sh min_dash_count 6
scripts/set_calibrator_param.sh roi_y0_pct 72
scripts/set_calibrator_param.sh roi_skew 8
scripts/set_calibrator_param.sh label sample
```

Comandos especiales:

```text
s=1  guardar raw/processed/mask/overlay + JSON
p=1  pausar/resumir
u=1  toggle undistort
q=1  salir
```

---

## ROIs de intersección

La detección tiene dos niveles:

1. **Entrada de intersección**: banda roja baja. Solo esta zona dispara el estado
   `READ_OPTIONS`.
2. **Opciones left/straight/right**: polígonos translúcidos superiores. Estos
   solo clasifican hacia dónde se puede ir.

Las ROIs de opciones ya no son rectángulos fijos para el conteo; ahora son
polígonos con skew ajustable para seguir mejor la perspectiva.

Parámetros útiles:

```text
roi_y0_pct / roi_y1_pct              banda roja de entrada
dynamic_option_roi                   1 = coloca opciones según la entrada detectada
entry_margin_pct                     separación vertical entre entrada y opciones
dynamic_option_height_pct            altura de la zona de opciones
option_gap_pct                       separación entre left/straight/right
straight_option_width_pct            ancho de la ROI central
option_roi_skew_pct                  diagonal de los polígonos
roi_skew                             alias de option_roi_skew_pct
rect_pct                             rectangularidad mínima de dash
max_aspect_x10                       forma máxima permitida de dash
min_dash_count                       cuántos dashes reales disparan intersección
stable_frames                        cuántos frames seguidos exige
```

Valores de arranque que usamos para explorar:

```bash
scripts/set_calibrator_param.sh roi_skew 8
scripts/set_calibrator_param.sh entry_margin_pct 15
scripts/set_calibrator_param.sh dynamic_option_height_pct 32
scripts/set_calibrator_param.sh option_gap_pct 6
scripts/set_calibrator_param.sh straight_option_width_pct 20
scripts/set_calibrator_param.sh rect_pct 35
scripts/set_calibrator_param.sh max_aspect_x10 45
```

Qué buscar visualmente:

- `dash` debe ser 6 si la entrada real de intersección tiene 6 cuadritos.
- Los cuadritos detectados se marcan con amarillo translúcido.
- Las ROIs laterales deben cubrir las diagonales de left/right sin agarrar fondo.
- La ROI central debe cubrir la rama recta hacia enfrente.
- Si una pieza de la pista tipo rompecabezas se convierte en falso dash, sube
  `rect_pct` o baja `max_aspect_x10` antes de cambiar la ROI.

---

## Archivos que sí se commitean

Después de calibrar:

```bash
git add config/camera_params.npz config/illumination_flatfield.npz
git commit -m "Recalibrate camera and illumination"
```

También conviene commitear los cambios de código/docs del flujo H264 y tuning.

No se commitean capturas crudas ni previews:

```text
calibration_images/
config/undistorted_preview.jpg
config/illumination_preview.jpg
```

---

## Comandos rápidos de diagnóstico

Cámara H264 cruda:

```bash
scripts/run_camera_h264_jetson.sh
```

Focus:

```bash
scripts/run_focus_assist_jetson.sh
```

Checkerboard:

```bash
scripts/run_checkerboard_capture_jetson.sh
scripts/run_calibrate_camera_jetson.sh
```

Iluminación:

```bash
scripts/run_illumination_calibrator_jetson.sh
```

Máscara/Otsu/ROIs:

```bash
scripts/run_line_calibrator_jetson.sh
```

Modo con sliders reales:

```bash
STREAM=local scripts/run_line_calibrator_jetson.sh
```
