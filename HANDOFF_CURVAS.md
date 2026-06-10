# Handoff: curvas Puzzlebot

## Contexto operativo

- Robot: Puzzlebot con Jetson + ROS2 Humble.
- Repo laptop: `/home/gnuno/dev_ws/src/manchester/puzzlebot-line-follower`
- Repo Jetson: `/home/puzzlebot/ros2_ws/src/puzzlebot_ros`
- Branch activa: `develop`
- La laptop no tiene ROS2 ni `cv2`; visión/nodo se validan en la Jetson.
- Para correr todo:

```bash
USE_SIGNS=1 NO_BUILD=1 scripts/run_demo_tmux.sh
```

- Para detener antes de relanzar:

```bash
scripts/stop_demo.sh
```

- Para sincronizar cambios de nodo/percepción:

```bash
scp /home/gnuno/dev_ws/src/manchester/puzzlebot-line-follower/puzzlebot_ros/line_follower.py puzzlebot@10.10.0.100:/home/puzzlebot/ros2_ws/src/puzzlebot_ros/puzzlebot_ros/line_follower.py
scp /home/gnuno/dev_ws/src/manchester/puzzlebot-line-follower/tools/control_panel.py puzzlebot@10.10.0.100:/home/puzzlebot/ros2_ws/src/puzzlebot_ros/tools/control_panel.py
ssh puzzlebot@10.10.0.100 'source /opt/ros/humble/setup.bash; cd /home/puzzlebot/ros2_ws && colcon build --packages-select puzzlebot_ros --symlink-install'
```

La WiFi de la Jetson es inestable; usar `ConnectTimeout=6` y reintentos.

## Estado reciente

El straight commit y el turn right llegaron a funcionar bien. El problema activo son curvas cerradas: el robot se descarrila porque el seguidor BEV a veces pierde o contamina la línea y acepta bordes/piezas del rompecabezas como si fueran la línea negra.

No asumir que es solo velocidad. En las sesiones malas se ven tres fallas:

1. El BEV acepta bases imposibles pegadas al borde del warp (`base_x` cerca de `0`, `30`, `327`) con confianza media/alta.
2. En una curva fuerte, el comando angular puede quedar demasiado bajo o incluso cambiar de signo por error/derivada/fallback.
3. Cuando se pierde la línea en curva, el modo `RECOVER` antes usaba `last_error`; ese error podía estar contaminado por fallback y buscar al lado contrario.

## Sesiones clave a pasar a la siguiente IA

Usar estas rutas:

```text
/home/gnuno/dev_ws/src/manchester/puzzlebot-line-follower/datasets/follower_session/20260609_193546
/home/gnuno/dev_ws/src/manchester/puzzlebot-line-follower/datasets/follower_session/20260609_194256
/home/gnuno/dev_ws/src/manchester/puzzlebot-line-follower/datasets/follower_session/20260609_195018
```

La más importante para el estado actual es:

```text
/home/gnuno/dev_ws/src/manchester/puzzlebot-line-follower/datasets/follower_session/20260609_195018
```

Hallazgos de `20260609_195018`:

- `22.07s`: `base_x=31`, `conf=0.50`, `v=0.097`. Esto está pegado al borde del BEV, no debe aceptarse como línea.
- Después aparecen `base_x=0/3/4/11/327` con confianza media/alta.
- En curva fuerte se vio `curv=+1.00` pero `w=-0.018`, o sea giro contrario.

Comando útil para inspeccionar:

```bash
python3 - <<'PY'
import csv, collections
p='datasets/follower_session/20260609_195018/controller_data.csv'
with open(p) as f:
    rows=list(csv.DictReader(f))
print(collections.Counter(r['state'] for r in rows))
for r in rows:
    if r.get('state') not in ('FOLLOW','RECOVER: line lost','ADVANCE','READ: decision'):
        continue
    try:
        t=float(r['t']); conf=float(r['conf'] or 0); curv=float(r['curv'] or 0)
        v=float(r['v'] or 0); w=float(r['w'] or 0); err=float(r['error'] or 0)
        base=float(r['base_x']) if r['base_x'] else 999
    except Exception:
        continue
    if r['state']!='FOLLOW' or conf<.6 or abs(curv)>.75 or abs(err)>65 or abs(w)>.18 or base<120 or base>285:
        print(f"t={t:6.2f} {r['state'][:7]:7s} conf={conf:.2f} curv={curv:+.2f} base={base:6.1f} err={err:+7.1f} v={v:.3f} w={w:+.3f}")
PY
```

## Commits recientes relevantes

- `f8752aa fix: bridge curve dropouts`
  - Agregó `lane_curve_dropout_s=2.0`
  - Agregó `curve_min_v=0.045`
  - Objetivo: no caer tan rápido al fallback durante una curva y evitar velocidad inútil cerca del deadband.

- `c13ce46 fix: recover curves by heading memory`
  - `RECOVER` usa memoria de curvatura antes que `last_error`.
  - Agregó `lane_curve_refresh_conf=0.35` para refrescar curva con fits débiles pero coherentes.
  - Bloquea fallback legacy durante pérdida reciente de curva.

- `a5f0bfc fix: reject curve edge locks`
  - Agregó `lane_base_edge_margin_pct=12` para rechazar bases BEV pegadas al borde.
  - Agregó `lane_curve_min_turn_w=0.075` para no mandar giro contrario/demasiado débil en curva fuerte.

## Parámetros nuevos en panel

En `tools/control_panel.py`, página `AntiZebra`:

- `lane_curve_dropout_s`
  - Default: `2.0`
  - Si cae al fallback muy pronto en curva, subir a `2.3` o `2.5`.
  - Si se queda demasiado “casado” con curva vieja al salir, bajar a `1.5`.

- `lane_curve_refresh_conf`
  - Default: `0.35`
  - Permite mantener memoria de curva con BEV débil.
  - Si no refresca memoria, bajar a `0.30`.
  - Si empieza a guardar curvas falsas, subir a `0.45`.

- `lane_base_edge_margin_pct`
  - Default: `12`
  - Rechaza `base_x` pegado al borde del warp.
  - En `20260609_195018`, con `12%` rechazaba los frames malos `base=31,30,0,327`.
  - Si rechaza línea real en curva, bajar a `10`.
  - Si todavía agarra borde, subir a `14`.

- `lane_curve_min_turn_w`
  - Default: `0.075`
  - Mínimo angular en curva fuerte según signo de curvatura reciente.
  - Si la curva abre demasiado, subir a `0.09`.
  - Si sobre-gira o zigzaguea, bajar a `0.055`.

En `Drive`:

- `curve_min_v`
  - Default: `0.045`
  - Piso de velocidad en curva fuerte.
  - Si sigue muy lento, subir a `0.050`.
  - Si se abre por ir rápido, bajar a `0.040`.

## Archivos principales

- `puzzlebot_ros/line_follower.py`
  - Máquina de estados, control PD, recover, guards de curva.
  - Revisar zona `LINE PERCEPTION + BASE CONTROL` y `PD Math`.

- `puzzlebot_ros/perception/lane.py`
  - BEV, máscara negra, sliding window, `base_x`, `curvature_norm`.
  - Si sigue aceptando bordes, probablemente el siguiente cambio debe ir aquí, no solo en el controlador.

- `tools/control_panel.py`
  - Knobs en vivo.

## Recomendación técnica para el siguiente paso

Si `a5f0bfc` todavía falla, no seguir metiendo parches al PD sin mirar imágenes. La siguiente IA debería:

1. Tomar frames JPG de la sesión fallida nueva y ubicar visualmente qué está marcando como línea cuando `base_x` se va a borde.
2. En Jetson, correr un script offline que procese esos frames con `analyze_lane` y guarde composites BEV con:
   - máscara negra
   - ventanas del sliding window
   - `base_x`
   - `eval_x`
   - `far_x`
   - razón de rechazo (`edge`, `jump`, `confidence`, etc.)
3. Si el borde se sigue colando, endurecer `perception/lane.py`:
   - rechazar fit si los pixeles usados están muy pegados al borde del warp
   - exigir continuidad vertical real en curvas
   - penalizar fit con ventanas saltando horizontalmente demasiado
   - no aceptar `base_x` extremo aunque `confidence` sea alta
4. Solo después retocar `kp`, `ff_gain`, `lane_curve_min_turn_w` o velocidad.

## Cosas que no conviene repetir

- No volver a activar `lane.dual_line`; empeoró mucho.
- No volver a meter lookahead/anticipación agresiva global sin guardias; ya empeoró curvas.
- No subir velocidad general antes de resolver edge locks; cuando acepta borde, más velocidad solo lo saca más rápido.
- No confiar en `last_error` para recover durante curva; ya se comprobó que puede estar contaminado y girar al lado contrario.

## Validación mínima después de cualquier cambio

En laptop:

```bash
python3 -m py_compile puzzlebot_ros/line_follower.py tools/control_panel.py
```

En Jetson:

```bash
ssh -o ConnectTimeout=6 puzzlebot@10.10.0.100 'python3 -m py_compile /home/puzzlebot/ros2_ws/src/puzzlebot_ros/puzzlebot_ros/line_follower.py /home/puzzlebot/ros2_ws/src/puzzlebot_ros/tools/control_panel.py'
ssh -o ConnectTimeout=6 puzzlebot@10.10.0.100 'source /opt/ros/humble/setup.bash; cd /home/puzzlebot/ros2_ws && colcon build --packages-select puzzlebot_ros --symlink-install'
```

Commit corto, Conventional Commits, sin `Co-Authored-By`, push a `origin/develop`.
