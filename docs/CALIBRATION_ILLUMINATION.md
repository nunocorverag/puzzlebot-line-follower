# Calibración de iluminación (flat-field) — "los rojos"

Corrige el **tinte rojizo** y el **viñeteo** (esquinas más oscuras) de la cámara.
El sensor no responde igual en todos los píxeles ni en todos los canales, así que
una superficie blanca se ve con una **mancha rojiza** y bordes oscuros. Esta
calibración mide ese patrón sobre una **lona/hoja blanca uniforme** y construye un
**mapa de ganancia** por píxel y canal que lo aplana. Resultado: blancos parejos,
sin tinte, y máscaras de color/línea mucho más estables.

> Se hace **DESPUÉS** de la calibración de cámara
> ([CALIBRATION_CHECKERBOARD.md](CALIBRATION_CHECKERBOARD.md)), porque el mapa se
> mide sobre imágenes **ya sin distorsión** (las mismas que verá el runtime).

---

## Qué necesitas

- Una superficie **blanca, mate y uniforme** que llene todo el cuadro: la **lona**
  blanca, una hoja grande o una pared blanca limpia. Mate, no brillante (el brillo
  causa reflejos especulares).
- La **misma luz** que tendrás en la pista. Si calibras con otra luz, no sirve.
- Luz **difusa y pareja**, sin sombras ni focos directos.

---

## Cómo tomar las muestras (robustez)

Es **auto-guiada**, pero ahora **no empieza a capturar hasta que presionas
Enter**. Primero acomodas la lona mirando el H264; cuando se vea bien, presionas
Enter, quitas manos/sombras, espera unos segundos y el tool junta varios frames
buenos y los **promedia** (baja el ruido) antes de calcular. Solo acepta frames
que pasan el filtro de calidad; el overlay te dice qué corregir:

1. **Llena el cuadro con la lona** — nada de fondo, ni bordes de la lona, ni
   objetos. Solo blanco.
2. **Sin sombras** — ni la tuya ni la del robot sobre la lona. Si estar
   totalmente de frente mete sombra, usa un **ángulo leve** hacia la lona. Para
   flat-field importa más que todo el cuadro vea blanco uniforme que estar
   perfectamente perpendicular. Si hay una zona oscura, dirá *"sombra detectada:
   ilumina parejo"*.
3. **Sin brillos especulares** — si un foco se refleja, cambia el ángulo. Una
   inclinación leve suele ayudar. Dirá *"brillo especular: cambia el angulo"*.
4. **Exposición correcta** — ni muy oscuro ni quemado. Dirá *"muy oscuro"* o
   *"muy brillante"*. Busca un gris-blanco parejo, no blanco quemado.
5. **Quieto** — mantén la cámara firme mientras junta los frames.
6. **Presiona Enter solo cuando estés listo** — después de Enter hay una pequeña
   espera para retirar la mano y evitar que tu sombra entre al promedio.

Cuando junta los frames objetivo (por defecto 25) calcula, guarda y termina solo.

---

## Pasos

```bash
# Apunta a la lona blanca llenando el cuadro.
scripts/run_illumination_calibrator_jetson.sh
#    Mira el H264, acomoda la lona, presiona Enter cuando estés listo.
#    Opcional: FRAMES=30
#    AUTO_START=1 recupera el comportamiento anterior.
#    Junta frames buenos, calcula y guarda.
#    Trae config/illumination_flatfield.npz y config/illumination_preview.jpg
```

### Cómo verificar

El tool imprime un reporte de residual:

```
std BGR antes : 31.4 28.9 40.2
std BGR despues:  6.1  5.8  6.4
media BGR antes : 150.2 158.7 196.1     <- R alto = tinte rojizo
media BGR despues: 171.0 171.2 171.4    <- canales parejos = tinte removido
```

- La **std después** debe bajar bastante (imagen más "plana").
- Las **medias BGR después** deben quedar **parejas** entre sí (el rojo deja de
  dominar = se quitó la mancha rojiza).

Abre **`config/illumination_preview.jpg`** (izquierda promedio crudo | derecha
corregido): el lado derecho debe verse blanco **uniforme**, sin la mancha rojiza
ni esquinas oscuras.

### Guardar

```bash
git add config/illumination_flatfield.npz
git commit -m "Recalibrate illumination flat-field"
```

---

## Detalles técnicos

- Se trabaja a **640×480** y, si existe `config/camera_params.npz`, se aplica
  `undistort` antes de medir (consistente con el runtime).
- Ganancia = `media_del_canal / referencia_suavizada`, recortada a `[0.25, 4.0]`;
  se promedian ~25 frames buenos para reducir ruido.
- El runtime la aplica vía `apply_illumination_gain` (en
  `puzzlebot_ros/perception/camera.py`). El formato del `.npz` (clave `gain`) se
  mantiene, así que todos los consumidores existentes la usan sin cambios.
- Código: `tools/illumination_calibrator.py`.
