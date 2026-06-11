# Puzzlebot Line Follower - documentacion para presentacion

Este documento resume el proyecto completo del seguidor de linea para el
Puzzlebot: que problema resuelve, como esta organizada la solucion, como funciona
la percepcion/control, como se calibran los subsistemas y como se opera durante
una demostracion.

La intencion no es reemplazar los runbooks tecnicos existentes, sino tener una
narrativa completa para explicar el proyecto en una presentacion, con suficiente
detalle para defender las decisiones de diseno y repetir la calibracion.

---

## 1. Resumen del proyecto

El objetivo del proyecto es que un Puzzlebot pueda recorrer una pista tipo
Manchester/puzzle-mat de forma autonoma usando una camara CSI montada en una
Jetson. El robot debe:

- detectar y seguir la linea negra del carril;
- anticipar curvas y no salirse en secciones cerradas;
- detectar cruces/intersecciones con marcas punteadas tipo zebra;
- detenerse en la zona correcta para leer las opciones de salida;
- recibir una decision manual o automatica para girar a la izquierda, seguir
  recto o girar a la derecha;
- obedecer, de forma opcional, semaforos y senales de trafico;
- transmitir video, metricas y diagnosticos para poder ajustar el sistema en
  vivo.

La solucion final no depende de un unico algoritmo fragil. Esta dividida en
modulos: calibracion de camara, correccion de iluminacion, transformacion a vista
cenital, seguimiento de linea, deteccion de cruces, maquina de estados,
controlador de movimiento y herramientas de tuning.

---

## 2. Hardware y plataforma

### Robot

El robot utilizado es un Puzzlebot con base diferencial. La Jetson ejecuta el
nodo principal de ROS 2 y publica comandos de velocidad en `/cmd_vel`. El puente
de motores se comunica mediante micro-ROS o mediante los scripts de arranque
incluidos.

### Computadora de abordo

La Jetson es responsable de:

- leer la camara CSI a 640x480;
- aplicar correccion de lente e iluminacion;
- correr la percepcion en tiempo real;
- ejecutar la maquina de estados del comportamiento;
- publicar `/cmd_vel`;
- transmitir video de depuracion por MJPEG o H264;
- exponer parametros ROS para ajuste en vivo.

### Laptop

La laptop se usa como estacion de operacion:

- sincroniza codigo hacia la Jetson;
- compila el workspace remoto;
- lanza scripts por SSH;
- muestra el stream de video;
- ejecuta tuners y dashboards;
- recibe datasets y snapshots para analisis offline.

Los scripts asumen por defecto:

```bash
JETSON_USER=puzzlebot
JETSON_HOST=10.10.0.100
REMOTE_WS=/home/puzzlebot/ros2_ws
```

---

## 3. Estructura del repositorio

Los archivos mas importantes son:

| Ruta | Funcion |
| --- | --- |
| `puzzlebot_ros/line_follower.py` | Nodo principal `autonomous_racer`: camara, percepcion, control, cruces, semaforo, senales, streaming y logs. |
| `puzzlebot_ros/perception/lane.py` | Seguimiento de linea con vista cenital, mascara de linea, ventanas deslizantes y ajuste polinomial. |
| `puzzlebot_ros/perception/zebra.py` | Detector robusto de cruces en coordenadas de suelo, medido en centimetros. |
| `puzzlebot_ros/perception/intersection.py` | Detector legacy/auxiliar de dashes en imagen original y herramienta de calibracion de ROIs. |
| `puzzlebot_ros/perception/signs.py` | Detector YOLO de senales y mapeo de clases a acciones de manejo. |
| `puzzlebot_ros/traffic_light.py` | Nodo independiente de semaforo por HSV. |
| `config/camera_params.npz` | Intrinsecos y distorsion de la camara. |
| `config/illumination_flatfield.npz` | Mapa de ganancia para corregir tinte/vineteo. |
| `config/lane_params.json` | Parametros de vista cenital y seguimiento de linea. |
| `config/zebra_params.json` | Parametros metricos del detector de cruces. |
| `config/control_params.json` | Ganancias de control, curvas, intersecciones, semaforo y senales. |
| `tools/` | Calibradores, grabadores, evaluadores offline y tuners. |
| `scripts/` | Automatizacion para sincronizar, compilar, correr, calibrar y detener el sistema. |
| `docs/` | Runbooks y documentacion tecnica del proyecto. |

---

## 4. Flujo general del sistema

El nodo principal corre un ciclo aproximado de 30 Hz:

1. Captura un frame de la camara CSI.
2. Aplica undistortion si existe `camera_params.npz`.
3. Aplica flat-field si existe `illumination_flatfield.npz`.
4. Ejecuta percepcion:
   - linea en vista cenital;
   - zebra/cruce en vista cenital ancha;
   - semaforo por HSV;
   - senales YOLO si fueron activadas.
5. Actualiza la maquina de estados.
6. Calcula velocidad lineal `v` y angular `w`.
7. Aplica compuertas de seguridad: `drive_enable`, semaforo, STOP/give-way,
   estado de cruce y recuperacion.
8. Publica `/cmd_vel`.
9. Publica telemetria, `/lane_status`, prompts de interseccion y stream de video.

El robot arranca con el movimiento deshabilitado. Esto permite observar la
percepcion sin que el robot avance. Para permitir movimiento se publica en
`/drive_enable`, normalmente con:

```bash
scripts/set_drive_jetson.sh on
```

Para detenerlo:

```bash
scripts/set_drive_jetson.sh off
scripts/stop_demo.sh
```

---

## 5. Seguimiento de linea

### Idea principal

La pista se ve en perspectiva desde la camara. Si se procesa directamente la
imagen original, la linea cambia de grosor, escala y orientacion dependiendo de
la distancia. Para hacerlo mas robusto, el sistema transforma la parte baja de la
imagen a una vista cenital o bird's-eye view.

En esa vista:

- el suelo queda rectificado;
- la linea se vuelve casi vertical;
- las curvas se pueden aproximar con un polinomio;
- las marcas laterales o costuras del piso quedan separadas espacialmente;
- los pixeles se pueden relacionar con dimensiones reales de la pista.

El modulo principal es `puzzlebot_ros/perception/lane.py`.

### Pipeline de vision de linea

1. **Homografia**

   Se toma un trapezoide de la imagen original, definido por:

   - `src_top_y_pct`
   - `src_top_half_w_pct`
   - `src_bot_y_pct`
   - `src_bot_half_w_pct`

   y se transforma a un rectangulo de `warp_w x warp_h`.

2. **Normalizacion y mascara**

   Sobre la imagen rectificada se genera una mascara de pixeles negros:

   - desenfoque Gaussiano para reducir ruido;
   - CLAHE para mejorar contraste local;
   - umbral Otsu, umbral adaptativo o umbral fijo;
   - apertura morfologica para eliminar costuras del tapete;
   - filtro de nucleo grueso para conservar solo trazos suficientemente anchos.

3. **Busqueda de base**

   Se calcula un histograma en la parte baja del bird's-eye y se restringe la
   busqueda a una banda alrededor del centro. Esto evita que el robot se vaya con
   una linea lateral, una costura o un borde de la pista.

4. **Ventanas deslizantes**

   Desde la base detectada, el algoritmo sube por la imagen con ventanas
   verticales. Si hay suficientes pixeles negros dentro de una ventana, la ventana
   se recentra. Esto reconstruye el recorrido de la linea.

5. **Ajuste polinomial**

   Con los puntos detectados se ajusta:

   ```text
   x = a*y^2 + b*y + c
   ```

   De ahi se obtiene:

   - offset lateral de la linea;
   - curvatura;
   - pendiente/heading;
   - confianza de deteccion;
   - punto cercano y punto de lookahead.

6. **Salida al controlador**

   El controlador usa el offset y la curvatura para definir `w`, y reduce `v`
   cuando se aproxima una curva.

### Parametros actuales de linea

Valores actuales en `config/lane_params.json`:

| Parametro | Valor | Interpretacion |
| --- | ---: | --- |
| `src_top_y_pct` | 55 | Parte superior del trapezoide de suelo. |
| `src_top_half_w_pct` | 14 | Medio ancho superior del trapezoide. |
| `src_bot_y_pct` | 95 | Parte inferior, cerca del robot. |
| `src_bot_half_w_pct` | 42 | Medio ancho inferior del trapezoide. |
| `warp_w` | 400 | Ancho de la vista cenital. |
| `warp_h` | 600 | Alto de la vista cenital. |
| `mask_method` | 0 | 0 = Otsu global, 1 = adaptativo. |
| `use_clahe` | 1 | Ecualizacion local activa. |
| `window_half_w_pct` | 12 | Medio ancho de cada ventana. |
| `base_search_half_w_pct` | 26 | Banda central para buscar la base. |
| `continuity` | 1 | Usa la posicion previa para evitar saltos. |
| `eval_y_pct` | 72 | Donde se evalua el offset para control. |
| `lookahead_y_pct` | 45 | Punto lejano para anticipar curva. |
| `min_windows_conf_pct` | 40 | Confianza minima para aceptar el fit. |

### Control de linea

El control base es tipo PD:

```text
error = centro_deseado - centro_detectado
w = kp * error + kd * derivada
```

Encima de eso se agregan mecanismos para curvas:

- `ff_gain`: feedforward de curva usando diferencia entre punto cercano y lejano;
- `curve_slow_gain`: reduce velocidad al detectar curvatura;
- `curve_min_scale`: limite inferior de reduccion de velocidad;
- `blind_turn`: si se pierde la linea en una curva, avanza y gira hacia la
  direccion recordada hasta reacquirir la linea.

Valores actuales relevantes en `config/control_params.json`:

| Parametro | Valor | Uso |
| --- | ---: | --- |
| `kp` | 0.0018 | Ganancia proporcional. |
| `kd` | 0.0 | Ganancia derivativa. |
| `max_v` | 0.09 m/s | Velocidad maxima nominal. |
| `max_w` | 0.6 rad/s | Giro maximo. |
| `ff_gain` | 1.0 | Anticipacion de curva. |
| `curve_slow_gain` | 0.6 | Frenado por curva. |
| `curve_min_scale` | 0.7 | Escala minima de velocidad en curva. |
| `blind_turn_enabled` | true | Recuperacion de curvas al perder linea. |
| `blind_turn_w` | 0.45 rad/s | Giro durante blind turn. |
| `blind_turn_v` | 0.05 m/s | Avance durante blind turn. |
| `blind_pre_s` | 2.0 s | Avance recto antes de girar al perder linea. |

---

## 6. Deteccion de cruces/intersecciones

### Problema

La interseccion no es una linea continua, sino una fila de marcas punteadas. En
imagen original, esas marcas cambian de forma por perspectiva y se ven peor al
salir de una curva. Por eso se implemento un detector especializado en vista
cenital ancha.

### Detector principal: zebra BEV

El modulo `puzzlebot_ros/perception/zebra.py` usa una homografia mas ancha que la
del seguidor de linea. La razon es que la homografia de linea esta optimizada
para seguir el carril central, mientras que en un cruce hay que ver tambien los
laterales y salidas.

El detector trabaja en coordenadas de suelo:

1. Rectifica el suelo con una vista cenital ancha.
2. Detecta blobs negros.
3. Convierte dimensiones de pixeles a centimetros usando una escala calibrada.
4. Filtra blobs por tamano real esperado de los dashes.
5. Ajusta por RANSAC una fila transversal de dashes.
6. Calcula:
   - distancia a la fila (`distance_cm`);
   - angulo de la fila (`angle_deg`);
   - centro lateral (`row_center_cm`);
   - numero de dashes;
   - span lateral;
   - opciones de salida: izquierda, recto, derecha.

La ventaja de hacerlo en centimetros es que un dash cercano y uno lejano se
validan con la misma logica fisica, no con thresholds arbitrarios de imagen.

### Parametros actuales de zebra

Valores actuales en `config/zebra_params.json`:

| Parametro | Valor | Uso |
| --- | ---: | --- |
| `widen_kx` | 2.4 | Ensancha el trapezoide de linea para ver cruces completos. |
| `warp_w` | 720 | Ancho del BEV de zebra. |
| `warp_h` | 600 | Alto del BEV de zebra. |
| `px_per_cm_x` | 11.1 | Escala lateral. |
| `px_per_cm_y` | 23.6 | Escala longitudinal. |
| `min_short_cm` | 0.9 | Tamano minimo del lado corto de un dash. |
| `max_long_cm` | 7.0 | Tamano maximo del lado largo de un dash. |
| `min_area_cm` | 2.0 | Area minima. |
| `max_area_cm` | 22.0 | Area maxima. |
| `row_tol_cm` | 2.0 | Tolerancia para RANSAC de la fila. |
| `min_dashes` | 3 | Dashes minimos para fila visible. |
| `min_span_cm` | 7.0 | Span lateral minimo de la fila. |
| `stable_frames_needed` | 3 | Debounce temporal. |
| `slow_distance_cm` | 30.0 | Distancia para zona lenta. |
| `stop_distance_cm` | 6.0 | Distancia de paro objetivo. |

### Maquina de estados de interseccion

El comportamiento de cruces es una maquina de estados:

```text
FOLLOW -> ADVANCE -> WAIT/READ -> COMMIT -> FOLLOW
```

#### FOLLOW

El robot sigue la linea normalmente. Si el detector zebra ve un cruce estable y
dentro de la distancia de deteccion, entra a ADVANCE.

Parametros:

- `detect_distance_cm`: distancia para activar el cruce, actualmente 22 cm por
  default en el nodo.
- `intersection_slow_speed`: velocidad maxima cuando ya se ve zebra.

#### ADVANCE

El robot avanza hacia el cruce con una velocidad baja y controlada. En esta fase
se evita usar agresivamente la linea porque las marcas punteadas pueden confundir
al seguidor.

El objetivo es llegar a una posicion donde pueda leer las opciones de salida.
Puede detenerse al:

- cruzar la primera fila de dashes;
- ver un salto de distancia de zebra;
- alcanzar la distancia de lectura;
- llegar al timeout.

Parametros relevantes:

- `approach_speed`: velocidad de aproximacion;
- `read_distance_cm`: distancia de lectura;
- `read_cross_jump_cm`: salto que indica que se cruzo la primera fila;
- `read_after_entry_max_cm`: avance maximo despues de cruzar la entrada;
- `advance_center_gain`: correccion lateral con centro de zebra;
- `advance_lane_keep_gain`: asistencia suave con la linea.

#### WAIT/READ

El robot se detiene, publica un prompt en `/intersection_prompt` y espera una
decision:

```bash
scripts/set_intersection_jetson.sh left
scripts/set_intersection_jetson.sh straight
scripts/set_intersection_jetson.sh right
```

Tambien se puede publicar directamente:

```bash
ros2 topic pub --once /intersection_decision std_msgs/msg/String "{data: 'left'}"
```

Si las senales YOLO estan activas, una flecha puede dejar una decision pendiente
para el siguiente cruce.

#### COMMIT

La decision se ejecuta como una maniobra abierta o semiabierta: avanza y gira por
un tiempo minimo para cruzar la interseccion antes de regresar al seguimiento de
linea. La razon es que si el robot entrega el control demasiado pronto, puede
reacquirir una marca del cruce y no la rama correcta.

Parametros actuales:

| Parametro | Valor | Uso |
| --- | ---: | --- |
| `commit_speed` | 0.08 m/s | Velocidad durante commit. |
| `commit_turn_w` | 0.6 rad/s | Giro para izquierda/derecha. |
| `commit_duration` | 3.5 s | Tiempo maximo de giro. |
| `commit_duration_straight` | 6.0 s | Tiempo maximo recto. |
| `commit_min_s` | 2.0 s | Tiempo minimo antes de reacquirir. |
| `commit_straight_min_s` | 4.5 s | Minimo especial para recto. |
| `intersection_min_travel_m` | 0.25 m | Guardia contra doble deteccion. |

---

## 7. Semaforo

El semaforo se maneja como una supervision opcional. Por defecto el robot no
espera a ver verde para arrancar: avanza normalmente y solo actua si detecta un
rojo o amarillo valido. Esto evita que el robot quede bloqueado cuando el
semaforo no esta en el campo de vision.

La deteccion usa HSV y valida forma:

- segmentacion por color;
- area minima/maxima;
- circularidad;
- relacion de aspecto;
- porcentaje de relleno;
- validacion contra una placa gris alrededor del disco.

Parametros actuales relevantes:

| Parametro | Valor |
| --- | ---: |
| `traffic_light_roi_y_pct` | 55 |
| `traffic_light_min_area` | 80 |
| `traffic_light_max_area` | 5000 |
| `traffic_light_min_circularity` | 0.65 |
| `traffic_light_require_plate` | true |
| `traffic_light_action_min_distance_cm` | 12 |
| `traffic_light_action_max_distance_cm` | 45 |

Modos:

- `IGNORE_TRAFFIC_LIGHT=1`: ignora semaforo para pruebas.
- `traffic_light_optional=true`: comportamiento actual, solo obedece si lo ve.
- `traffic_light_optional=false`: modo estricto, requiere verde.

---

## 8. Senales de trafico con YOLO

Las senales estan implementadas como una funcion opcional y no bloqueante. Si el
modelo no carga o no se activa, el seguidor de linea sigue funcionando.

Archivo principal: `puzzlebot_ros/perception/signs.py`.

Modelo: `config/best.pt`.

Clases esperadas/mapeadas:

- `trabajadores` -> reduce velocidad por algunos segundos;
- `stop` -> alto temporal;
- `give-way` -> alto corto;
- `vuelta-izquierda` / `left` -> decision pendiente izquierda;
- `vuelta-derecha` / `right` -> decision pendiente derecha;
- `straight` / `recto` -> decision pendiente recto.

Se corre sobre la banda superior de la imagen para no confundir marcas del suelo
con senales. La inferencia no corre cada frame, sino cada `every_n` frames para
no afectar el ciclo de control.

Activacion:

```bash
USE_SIGNS=1 scripts/run_line_follower_jetson.sh
```

Parametros relevantes:

| Parametro | Valor |
| --- | ---: |
| `signs_conf` | 0.55 |
| `workers_speed_factor` | 0.5 |
| `workers_slow_s` | 4.0 |
| `stop_seconds` | 3.0 |
| `giveway_seconds` | 1.5 |
| `sign_act_area_pct` | 6.0 |
| `sign_turn_act_area_pct` | 1.4 |
| `sign_cooldown_s` | 6.0 |
| `sign_forget_s` | 15.0 |

---

## 9. Calibracion: metodologia completa

La calibracion se hace por capas. Es importante respetar el orden porque cada
capa depende de la anterior.

Orden recomendado:

1. enfoque de lente;
2. intrinsecos de camara;
3. flat-field de iluminacion;
4. montaje/tilt;
5. homografia de linea;
6. escala metrica del BEV;
7. detector zebra;
8. controlador de linea;
9. curvas y recuperacion;
10. intersecciones;
11. semaforo y senales.

### 9.1 Enfoque

Antes de calibrar, se ajusta fisicamente el lente. Cambiar el enfoque despues
puede alterar ligeramente los intrinsecos.

Comando:

```bash
scripts/run_focus_assist_jetson.sh
```

Metodologia:

1. Colocar un objetivo con textura o texto a la distancia de trabajo.
2. Girar el lente lentamente.
3. Maximizar el valor de nitidez que muestra el asistente.
4. Fijar el lente y no volver a moverlo.

### 9.2 Intrinsecos de camara

Archivo resultante:

```text
config/camera_params.npz
```

Objetivo:

- calcular matriz `K`;
- calcular coeficientes de distorsion;
- permitir `cv2.undistort`;
- estabilizar la geometria para homografias.

Comandos:

```bash
scripts/run_checkerboard_capture_jetson.sh
scripts/run_calibrate_camera_jetson.sh
```

Metodologia:

1. Usar un checkerboard plano y rigido.
2. Capturar 20-30 vistas variadas.
3. Cubrir centro, esquinas y bordes del frame.
4. Incluir distancias cercana, media y lejana.
5. Incluir inclinaciones en pitch, yaw y roll.
6. Evitar reflejos, blur y sombras fuertes.

Criterio de calidad:

- RMS menor a 0.5 px: excelente;
- RMS entre 0.5 y 1.0 px: aceptable;
- RMS mayor a 1.0 px: repetir captura.

Resultado validado en la documentacion existente:

```text
30/30 imagenes detectadas
RMS reprojection error: 0.3449 px
Veredicto: EXCELLENT
pattern: 5x7
image_size: 640x480
```

### 9.3 Calibracion de iluminacion

Archivo resultante:

```text
config/illumination_flatfield.npz
```

Objetivo:

- corregir tinte rojizo;
- corregir vignetting;
- hacer mas estables las mascaras de color y negro.

Comando:

```bash
scripts/run_illumination_calibrator_jetson.sh
```

Metodologia:

1. Apuntar la camara a una superficie blanca, mate y uniforme.
2. Llenar todo el frame con esa superficie.
3. Evitar sombras de manos, robot o ambiente.
4. Evitar brillos especulares.
5. Presionar Enter cuando la imagen se vea uniforme.
6. El sistema captura y promedia frames validos.
7. Se genera un mapa de ganancia por pixel y por canal.

Criterio de calidad:

- la desviacion estandar por canal debe bajar claramente;
- el promedio BGR final debe quedar balanceado;
- el preview corregido debe verse blanco uniforme.

Resultado documentado:

```text
std BGR before: 3.4 4.4 14.5
std BGR after : 0.6 0.6 0.8
```

### 9.4 Montaje y tilt

Archivo:

```text
config/camera_pose.json
```

Valor actual:

```json
{
  "pitch_deg": 11.1
}
```

La homografia solo es valida para una pose fija de camara. Si cambia altura o
inclinacion, se debe recalibrar la homografia y posiblemente el flat-field. No
es necesario repetir intrinsecos si no se toca el enfoque ni se cambia la camara.

Comando:

```bash
scripts/run_tilt_assistant_jetson.sh
scripts/set_tilt_param.sh save 1
```

Criterio practico:

- parte baja del frame: suelo, linea y zebra;
- parte alta del frame: semaforo, senales y vista adelantada;
- evitar apuntar demasiado hacia abajo porque se perderian senales/semaforos.

### 9.5 Homografia de linea

Archivo:

```text
config/lane_params.json
```

Comando:

```bash
scripts/run_warp_calibrator_jetson.sh
```

Ajustes desde otra terminal:

```bash
scripts/set_warp_param.sh src_top_y_pct 55
scripts/set_warp_param.sh src_top_half_w_pct 14
scripts/set_warp_param.sh src_bot_y_pct 95
scripts/set_warp_param.sh src_bot_half_w_pct 42
scripts/set_warp_param.sh save_lane 1
```

Metodologia:

1. Colocar el robot frente a un tramo recto.
2. Verificar que la linea recta se vea vertical en el BEV.
3. Ajustar el trapezoide hasta que las lineas paralelas se mantengan paralelas.
4. Confirmar que la zona inferior cubre la linea cercana al robot.
5. Confirmar que la zona superior no invade demasiado el espacio de senales.
6. Guardar cuando el BEV sea estable.

### 9.6 Escala metrica

La escala se usa especialmente para zebra. Las medidas reales documentadas de la
pista son:

| Medida | Valor |
| --- | ---: |
| Ancho de linea | 2.2 cm |
| Ancho de carril, borde a centro | 11.8 cm |
| Dash longitudinal | 2.2 cm |
| Dash transversal | 3.15 cm |
| Gap entre dashes | 0.8 cm |
| Profundidad de interseccion | 26.1 cm |
| Distancia entre intersecciones cercanas | 9.8 cm |

Comando auxiliar:

```bash
tools/measure_ground_scale.py
```

En la configuracion actual:

```text
px_per_cm_x = 11.1
px_per_cm_y = 23.6
```

### 9.7 Calibracion de zebra/interseccion

Archivo:

```text
config/zebra_params.json
```

Herramientas utiles:

```bash
scripts/run_line_calibrator_jetson.sh
tools/zebra_module_eval.py
tools/eval_zebra.py
tools/bev_zebra_preview.py
```

Metodologia:

1. Capturar ejemplos de recta, curva e interseccion.
2. Ver que el BEV ancho incluya toda la fila de dashes.
3. Ajustar escala px/cm usando dashes reales.
4. Filtrar blobs por dimensiones reales:
   - area minima/maxima;
   - lado corto minimo;
   - lado largo maximo.
5. Ajustar RANSAC:
   - tolerancia de fila;
   - dashes minimos;
   - span lateral minimo.
6. Validar que `distance_cm` disminuye de forma estable al acercarse.
7. Ajustar `slow_distance_cm` y `stop_distance_cm`.
8. Validar salidas izquierda/recto/derecha solo cuando el robot esta bien
   alineado.

### 9.8 Tuning del controlador

Herramienta recomendada:

```bash
scripts/run_param_tuner_jetson.sh
```

Flujo:

1. Arrancar motor agent.
2. Arrancar follower con video.
3. Levantar ruedas y activar `drive_enable`.
4. Ajustar velocidad maxima realista.
5. En recta, ajustar `kp` hasta que siga sin zigzaguear.
6. Si oscila, bajar `kp`, subir `kd` o leer mas adelante bajando
   `lane.eval_y_pct`.
7. En curva, ajustar feedforward, desaceleracion y blind turn.
8. Guardar parametros con la tecla `s`.

Reglas de diagnostico:

| Sintoma | Ajuste recomendado |
| --- | --- |
| Zigzag en recta | Bajar `kp`, subir `kd`, bajar `lane.eval_y_pct`. |
| Se abre en curva | Subir `ff_gain`, bajar velocidad o subir `max_w`. |
| Gira muy brusco | Bajar `max_w` o bajar `kp`. |
| Pierde linea en curva | Ajustar `blind_turn_w`, `blind_turn_v`, `blind_pre_s`. |
| Se confunde con zebra | Revisar anti-zebra row reject y lane hold. |
| Reacciona tarde | Leer mas adelante con `lane.eval_y_pct` menor. |

### 9.9 Tuning de intersecciones

Flujo recomendado:

1. Validar que zebra se detecta a distancia estable.
2. Ajustar `detect_distance_cm` para entrar a ADVANCE a tiempo.
3. Ajustar `approach_speed` para que el robot se acerque sin saltos.
4. Ajustar `read_distance_cm`, `read_cross_jump_cm` y `read_after_entry_max_cm`
   hasta que se detenga en la ventana correcta.
5. Validar lectura de opciones.
6. Ajustar `commit_turn_w` y `commit_duration` para giros de 90 grados.
7. Ajustar `commit_min_s` y `commit_straight_min_s` para que no reacquiera las
   marcas del cruce antes de salir.
8. Probar doble interseccion y ajustar `intersection_min_travel_m`.

---

## 10. Operacion para demo

### Arranque rapido

Desde la laptop:

```bash
scripts/sync_to_jetson.sh
scripts/build_on_jetson.sh
scripts/run_demo_tmux.sh
```

O en terminales separadas:

```bash
# Terminal 1
scripts/run_motor_agent_jetson.sh

# Terminal 2
IGNORE_TRAFFIC_LIGHT=1 scripts/run_line_follower_jetson.sh

# Terminal 3
scripts/run_param_tuner_jetson.sh

# Terminal 4
scripts/set_drive_jetson.sh on
```

### Video

Stream MJPEG:

```text
http://10.10.0.100:8080
```

Tambien existe modo H264 para menor latencia/ancho de banda, activado por los
scripts correspondientes.

### Comandos importantes

```bash
scripts/set_drive_jetson.sh on
scripts/set_drive_jetson.sh off
scripts/set_intersection_jetson.sh left
scripts/set_intersection_jetson.sh straight
scripts/set_intersection_jetson.sh right
scripts/set_intersection_jetson.sh reset
scripts/stop_demo.sh
```

### Seguridad

Reglas de operacion:

- arrancar siempre con ruedas levantadas si se esta ajustando control;
- verificar que solo hay un follower corriendo;
- mantener `drive_enable` apagado mientras se calibra vision;
- usar `scripts/stop_demo.sh` al terminar o si algo se comporta raro;
- para tuning, cambiar un parametro a la vez.

---

## 11. Telemetria y depuracion

El sistema expone varios canales:

| Canal | Uso |
| --- | --- |
| `/cmd_vel` | Comando final de velocidad. |
| `/drive_enable` | Habilitacion segura de movimiento. |
| `/lane_status` | `[off, conf, curv, v, w]` para tuner/graficas. |
| `/intersection_prompt` | Prompt textual cuando espera decision. |
| `/intersection_decision` | Entrada de decision de cruce. |
| `/intersection_reset` | Reset de estado de interseccion. |
| `/robot_vel` | Velocidad medida, si llega del motor agent. |

El video de depuracion incluye:

- frame original corregido;
- HUD de estado;
- BEV de linea con mascara, ventanas y fit;
- BEV ancho de zebra con dashes, fila y razones de opciones;
- overlays de semaforo y senales si aplican.

Tambien se pueden activar logs:

```bash
CONTROLLER_LOG=1 scripts/run_line_follower_jetson.sh
```

Esto genera CSV con:

- estado;
- offset;
- confianza;
- curvatura;
- distancia zebra;
- opciones;
- velocidades;
- ganancias;
- estado de blind turn y commit.

---

## 12. Decisiones de diseno importantes

### Vista cenital en vez de imagen cruda

La vista cenital reduce dependencia de perspectiva y permite usar medidas
fisicas. Esto fue clave para curvas e intersecciones.

### Detector zebra separado del detector de linea

La linea y la zebra tienen geometria distinta. Usar un unico detector causaria
confusiones: la zebra puede parecer una linea horizontal, y la linea puede
desaparecer entre dashes. Separarlos simplifica cada problema.

### Parametros vivos y persistentes

Los parametros se pueden ajustar en vivo por ROS y luego guardar a JSON. Esto
evita recompilar para cada cambio y permite hacer tuning directamente sobre el
robot.

### `drive_enable` como compuerta de seguridad

El nodo puede correr percepcion y streaming sin publicar movimiento continuo. Es
una separacion importante entre observar y actuar.

### Comportamientos opcionales no bloqueantes

Semaforo y senales no deben romper el seguidor de linea. Por eso:

- el semaforo puede ser opcional;
- YOLO esta apagado por default;
- si el modelo no carga, se degrada a "no hay senal";
- las senales solo modifican velocidad, alto o decision pendiente.

### Commit de interseccion con tiempo minimo

Aunque el robot pueda ver una linea justo despues de iniciar el giro, no conviene
regresar inmediatamente al seguidor. Primero debe cruzar la zona de dashes para
no engancharse con marcas equivocadas.

---

## 13. Limitaciones conocidas y trabajo futuro

Limitaciones actuales:

- La homografia depende de que la camara no se mueva.
- Las curvas muy cerradas siguen limitadas por la fisica: `R_min = v / max_w`.
- Las decisiones de ruta aun son manuales o por senales, no por un mapa global.
- La lectura de opciones de cruce depende de llegar suficientemente alineado.
- El semaforo por HSV puede requerir tuning si cambia mucho la iluminacion.

Mejoras futuras:

- mapa topologico de la pista para decidir rutas automaticamente;
- estimacion de pose/odom mas confiable para commits por distancia real;
- clasificador de semaforo por posicion del disco dentro de la placa;
- dataset mas amplio para validar bajo distintas iluminaciones;
- pruebas automatizadas con frames etiquetados para regresion de percepcion.

---

## 14. Guion sugerido para presentacion

1. **Problema**
   - Seguir linea no basta: hay curvas, cruces, semaforos y senales.
   - La camara ve perspectiva, ruido e iluminacion variable.

2. **Arquitectura**
   - Jetson + ROS 2 + camara CSI.
   - Nodo principal modular.
   - Configuracion persistente en JSON/NPZ.

3. **Calibracion**
   - Enfoque.
   - Intrinsecos.
   - Flat-field.
   - Homografia.
   - Escala metrica.

4. **Percepcion**
   - Bird's-eye lane follower.
   - Sliding windows + polinomio.
   - Zebra detector en centimetros.
   - Semaforo HSV.
   - Senales YOLO.

5. **Control**
   - PD con feedforward.
   - Reduccion de velocidad en curvas.
   - Blind turn para reacquirir linea.

6. **Intersecciones**
   - FOLLOW -> ADVANCE -> WAIT -> COMMIT.
   - Lectura de opciones.
   - Decision manual o por senal.

7. **Demo**
   - Mostrar HUD.
   - Activar `drive_enable`.
   - Mostrar cruce y decision.
   - Detener con `stop_demo.sh`.

8. **Conclusiones**
   - Separar calibracion, percepcion y control hizo el sistema ajustable.
   - La vista cenital y las medidas metricas aumentaron robustez.
   - El sistema quedo listo para extenderse con mapa topologico.

---

## 15. Referencias internas

Documentos tecnicos relacionados:

- `README.md`
- `docs/RUNBOOK.md`
- `docs/LANE_FOLLOWING.md`
- `docs/PERCEPTION_TUNING.md`
- `docs/CALIBRATION_CHECKERBOARD.md`
- `docs/CALIBRATION_ILLUMINATION.md`
- `docs/SETUP.md`
- `docs/SCRIPTS.md`
- `docs/HANDOFF_2026-06-08.md`

Comandos principales:

```bash
scripts/sync_to_jetson.sh
scripts/build_on_jetson.sh
scripts/run_demo_tmux.sh
scripts/run_line_follower_jetson.sh
scripts/run_param_tuner_jetson.sh
scripts/run_line_calibrator_jetson.sh
scripts/run_warp_calibrator_jetson.sh
scripts/run_illumination_calibrator_jetson.sh
scripts/run_checkerboard_capture_jetson.sh
scripts/run_calibrate_camera_jetson.sh
scripts/stop_demo.sh
```
