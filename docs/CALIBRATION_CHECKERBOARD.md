# Calibración de cámara (tablero de ajedrez)

Calcula los **intrínsecos** de la cámara CSI: la matriz `K` (distancia focal `fx,fy`
y centro óptico `cx,cy`) y los coeficientes de **distorsión** del lente. Con eso
`cv2.undistort` endereza las líneas curvadas por el lente, y toda la geometría
aguas abajo (seguidor de línea, detección de intersecciones) queda correcta.

> Se hace **primero**. La calibración de iluminación
> ([CALIBRATION_ILLUMINATION.md](CALIBRATION_ILLUMINATION.md)) va **después**,
> porque usa imágenes ya sin distorsión.

El `config/camera_params.npz` anterior venía de **otra cámara**, por eso hay que
rehacerlo con esta CSI en su pose montada.

---

## Paso 0: enfoque (antes de calibrar)

Ajusta el **enfoque** del lente primero y **no lo toques después** (cambiar el
enfoque altera ligeramente los intrínsecos). El asistente de enfoque te lo mide
en vivo por H264 — apunta a un objetivo con textura (el tablero o texto impreso)
a la distancia de trabajo y **gira el lente para MAXIMIZAR** el número:

```bash
scripts/run_focus_assist_jetson.sh
```

La barra/numero muestran la nitidez vs el pico; cuando dice **"EN EL PICO"**
quedó enfocado. No hay un número "bueno" universal: solo maximizas sobre una
escena fija. Ya con el foco fijo, sigue con la captura del tablero.

## Qué necesitas

- El tablero impreso y **pegado plano sobre algo rígido** (tu cartón ya sirve).
  Si se dobla, la calibración sale mal.
- Tu tablero: **6×8 cuadros → patrón `5x7` esquinas internas** (las cruces donde
  se tocan 4 cuadros, sin contar el borde). El tool también auto-detecta por si
  lo cuentas en otra orientación.
- Buena luz **pareja**, sin reflejos/brillos sobre el papel.

---

## Cómo tomar las muestras (esto es lo que la hace robusta)

La captura es **auto-guiada**: ves el stream H264 en la laptop y mueves el
tablero siguiendo las pistas en pantalla. El tool **solo guarda** fotos que están
nítidas, quietas y que **aportan una pose nueva**. Necesitas variedad en 3 ejes:

1. **Posición en el cuadro** — lleva el tablero a las **4 esquinas y al centro**
   del encuadre (cubre la grilla 3×3). Así calibras bien los bordes del lente,
   donde más distorsiona.
2. **Distancia** — toma **cerca, media y lejos** (que el tablero ocupe mucho,
   medio y poco del cuadro). Pero que **siempre se vea completo**.
3. **Inclinación** — inclínalo **±30–45°** hacia arriba, abajo, izquierda y
   derecha, y **gíralo en el plano**. Nunca todas las fotos de frente y planas:
   eso da una calibración pobre.

Reglas de oro:
- **Muévete despacio y detente** un instante en cada pose (si hay movimiento, la
  foto sale borrosa y el tool la rechaza: dirá "borroso").
- **Ideal: 20–30 vistas variadas.** Es el rango óptimo (consenso OpenCV/MATLAB).
  De 30 a 50 ganas muy poco; pasar de ~50 ya no mejora. **Calidad > cantidad:**
  15 vistas excelentes valen más que 60 mediocres. El default ya es `TARGET=30`.
- **No necesitas simetría/paralelismo.** El cálculo es una optimización *global*;
  inclinar a la izquierda en un borde y no "espejearlo" a la derecha **no afecta**.
  Lo que importa es: tocar **todo el cuadro** (las 4 esquinas y bordes) e incluir
  inclinaciones en **ambos sentidos** en el total — no que coincidan con la posición.
- Evita **reflejos** y sombras duras sobre el tablero.
- El tablero debe ocupar buena parte del cuadro, pero **entero**, sin cortarse.

El overlay te dice qué falta: *"aleja el tablero"*, *"acerca el tablero"*,
*"gira en profundidad a la izquierda (16+ deg)"*, y el progreso
`18/30  dist 3/3  pose 4/6`. También muestra `front pitch`,
`front yaw` y `roll2d` en grados para que sepas cuánto estás inclinando.
`front pitch/yaw` cerca de 0 significa que el tablero está de frente a la cámara.

---

## Pasos

```bash
# 1) Captura auto-guiada (preview por H264 en la laptop). Wheels-up no aplica
#    (no mueve el robot). Mueve el tablero hasta llegar a la meta.
scripts/run_checkerboard_capture_jetson.sh
#    Default 30 capturas. Variables opcionales: TARGET=40  PATTERN=5x7
#    Termina solo al llegar a la meta, o corta con Ctrl+C.
#    Las imágenes se traen a ./calibration_images/ en la laptop.

# 2) Cómputo de la calibración (corre en el Jetson; OpenCV no está en la laptop).
scripts/run_calibrate_camera_jetson.sh
#    Trae de vuelta config/camera_params.npz y config/undistorted_preview.jpg
```

### Cómo leer el resultado

El paso 2 imprime:

```
RMS reprojection error: 0.34 px  (640x480)
Veredicto: EXCELENTE
```

- **RMS < 0.5 px** → excelente.
- **0.5–1.0 px** → aceptable.
- **> 1.0 px** → repite la captura con **más variedad** (sobre todo inclinaciones
  y esquinas del cuadro). El tool ya descarta automáticamente las peores imágenes
  (outliers) antes de reportar.

Abre **`config/undistorted_preview.jpg`** (izquierda original | derecha corregida):
las líneas rectas del entorno deben verse **rectas** en la versión corregida.

### Guardar

```bash
git add config/camera_params.npz
git commit -m "Recalibrate CSI camera intrinsics"
# (un sync posterior lo reempuja al Jetson; las imágenes crudas quedan en
#  calibration_images/, ignoradas por git)
```

---

## Detalles técnicos

- Se captura y calibra a **640×480**, la **misma** resolución del runtime, para
  que `K` sea válida sin reescalar (`line_follower`, `sign_detector`, etc. usan
  640×480).
- Detección con `cv2.findChessboardCornersSB` (robusta a blur/luz) con fallback
  al clásico + `cornerSubPix`. Cómputo con `cv2.calibrateCamera`.
- El tamaño físico del cuadro (`SQUARE_MM`) **no afecta a `K`** ni a la
  distorsión; solo se guarda como metadato. Puedes ignorarlo.
- Código: `tools/calib_capture_checkerboard.py` (captura) y
  `tools/calibrate_camera.py` (cómputo).
