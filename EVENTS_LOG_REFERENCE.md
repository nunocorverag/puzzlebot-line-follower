# Events Log Reference

Este documento describe todos los eventos que se guardan en `events.json` durante una sesión del robot.

## Eventos de Señales de Tráfico

### `signs_detected`
Se registra cada vez que se detectan señales (throttled a cada 2s).

**Campos:**
- `detections`: Lista de todas las señales detectadas
  - `name`: Nombre de la señal (corregido si hubo verificación)
  - `conf`: Confianza del modelo YOLO (0-1)
  - `area_pct`: Área de la caja como % del ROI
  - `score`: Score combinado (conf × área normalizada)
  - `corrected_from`: (opcional) Nombre original si hubo corrección
  - `votes`: (opcional) Detalles de votación de verificación
    - `all_votes`: Lista de votos de cada método
    - `vote_counts`: Conteo de votos por dirección
    - `winner`: Dirección ganadora
- `selected`: Nombre de la señal seleccionada (mejor score)
- `selected_conf`: Confianza de la señal seleccionada
- `selected_area`: Área de la señal seleccionada

**Ejemplo:**
```json
{
  "event": "signs_detected",
  "detections": [
    {
      "name": "turn_right",
      "conf": 0.842,
      "area_pct": 12.5,
      "score": 0.105,
      "corrected_from": "turn_left",
      "votes": {
        "all_votes": ["turn_right", "turn_right", "turn_right"],
        "vote_counts": {"turn_right": 3},
        "winner": "turn_right"
      }
    }
  ],
  "selected": "turn_right",
  "selected_conf": 0.842,
  "selected_area": 12.5
}
```

### `sign_action`
Se registra cuando una señal dispara una acción.

**Campos:**
- `sign_name`: Nombre de la señal
- `action`: Tipo de acción
  - `pending_turn`: Señal direccional (left/right/straight)
  - `slow_zone`: Zona de trabajadores
  - `stop_hold`: Stop o Give Way
- `conf`: Confianza de la detección
- `area_pct`: (opcional) Área de la caja
- `direction`: (para pending_turn) Dirección del giro
- `expires_s`: (para pending_turn) Tiempo de expiración
- `duration_s`: (para slow_zone/stop_hold) Duración de la acción

**Ejemplos:**
```json
{
  "event": "sign_action",
  "sign_name": "turn_right",
  "action": "pending_turn",
  "direction": "right",
  "conf": 0.842,
  "area_pct": 12.5,
  "expires_s": 8.0
}
```

```json
{
  "event": "sign_action",
  "sign_name": "workers",
  "action": "slow_zone",
  "conf": 0.91,
  "duration_s": 4.0
}
```

### `sign_timeout`
Se registra cuando un `pending_turn` expira por no ver la señal.

**Campos:**
- `discarded_turn`: Dirección que se descartó
- `timeout_s`: Tiempo de timeout configurado

**Ejemplo:**
```json
{
  "event": "sign_timeout",
  "discarded_turn": "left",
  "timeout_s": 8.0
}
```

## Eventos de Intersecciones

### `approach_start`
Inicio de fase APPROACH (avanzar hacia la intersección).

**Campos:**
- `dist_cm`: Distancia a la zebra en cm
- `came_straight`: Si llegó recto o desde curva
- `align_prior`: Datos de alineación previa
  - `frames`: Frames de historia
  - `heading_deg`: Heading mediano
  - `abs_heading_deg`: Heading absoluto mediano
  - `curv`: Curvatura mediana
  - `off`: Offset mediano
  - `curved`: Si se considera que venía de curva

### `wait_start`
Inicio de fase WAIT (detenido en la intersección).

**Campos:**
- `dist_cm`: Distancia actual a la zebra
- `voted_options`: Opciones detectadas
- `option_votes`: Votos acumulados por opción

### `approach_timeout`
Timeout de la fase APPROACH.

### `commit_start`
Inicio del commit (cruzar la intersección).

**Campos:**
- `direction`: Dirección del commit (left/right/straight)
- `duration_s`: Duración máxima del commit
- `min_s`: Duración mínima antes de re-adquisición
- `pre_advance_cm`: Avance previo antes de girar

### `commit_end`
Fin del commit.

**Campos:**
- `direction`: Dirección del commit
- `reacquired`: Si re-adquirió la línea
- `timeout`: Si terminó por timeout

## Eventos de Decisiones

### `decision_received`
Decisión de intersección recibida del operador.

**Campos:**
- `decision`: Decisión normalizada (left/right/straight)

### `decision_ignored`
Decisión ignorada (formato inválido).

**Campos:**
- `raw`: Decisión raw recibida
- `reason`: Razón del rechazo

### `intersection_reset`
Reset manual del estado de intersección.

## Eventos de Sistema

### `recorder`
Estado del grabador de datos.

**Campos:**
- `enabled`: Si está grabando
- `snaps`: Número total de snapshots

## Estructura Base de Eventos

Todos los eventos incluyen:
- `t`: Timestamp relativo al inicio (segundos)
- `event`: Nombre del evento
- `state`: Estado del robot (FOLLOW/APPROACH/WAIT/COMMIT)
- `phase`: Fase de intersección (None/approach/wait)
- `commit`: Dirección de commit activo
- `pending`: Si hay intersección pendiente
- `decision`: Decisión de intersección
- `options`: Opciones detectadas en intersección
- `lane`: Estado de detección de línea
  - `detected`: Si detectó línea
  - `off`: Offset normalizado
  - `curv`: Curvatura normalizada
- `zebra`: Estado de detección de zebra
  - `seen`: Si ve zebra
  - `dist_cm`: Distancia en cm
  - `stable`: Frames estables

## Notas

- Los eventos de señales incluyen **detalles de votación** cuando hay verificación de dirección
- Los eventos están **throttled** para evitar spam (signs_detected cada 2s)
- Todos los valores numéricos están **redondeados** para reducir tamaño del JSON
- Los eventos se guardan en **`events.json`** en el directorio de la sesión
