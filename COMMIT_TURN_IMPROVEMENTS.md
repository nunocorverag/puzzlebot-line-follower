# Mejoras al Commit de Giros (Turn Left/Right)

## Problema Original
- Los giros (left/right) no avanzaban suficiente en línea recta antes de girar
- El robot podía agarrar bordes o líneas incorrectas durante el giro
- Re-adquisición de línea poco robusta

## Soluciones Implementadas

### 1. **Mayor Pre-Avance Antes de Girar** 📏
**Parámetro:** `commit_turn_pre_advance_cm`
- **Antes:** 4.0 cm
- **Ahora:** 10.0 cm

**Beneficio:** El robot avanza más en línea recta antes de iniciar el giro, mejorando el posicionamiento para tomar la curva correctamente.

**Flujo:**
```
1. Inicia commit left/right
2. Avanza 10cm en línea recta (W=0.0)
3. Luego inicia el giro (W=±0.6)
```

### 2. **Re-Adquisición Robusta para Giros** 🎯

**Criterios Diferenciados:**

#### Para Giros (left/right):
```python
reacquired = (
    lane.detected AND
    lane.confidence >= 0.7 AND    # Alta confianza (antes 0.5)
    abs(lane.offset) < 0.6         # Offset razonable (NUEVO)
)
```

#### Para Straight (sin cambios):
```python
reacquired = (
    lane.detected AND
    lane.confidence >= 0.5
)
```

**Beneficios:**
- ✅ **Confianza alta (0.7)**: Asegura que es una línea real, no ruido
- ✅ **Offset limitado (< 0.6)**: Evita agarrar bordes de la intersección
- ✅ **Previene detecciones falsas**: No termina el giro prematuramente

### 3. **Logging Detallado para Debug** 📊

**Nuevo logging durante commit de giros:**
```
[INTERSECTION] Committing left: V=0.08, W=0.60, pre=7/10cm, lane=conf=0.45,off=0.82, reacq=False
[INTERSECTION] Committing left: V=0.08, W=0.60, pre=10/10cm, lane=conf=0.75,off=0.23, reacq=True
```

**Información mostrada:**
- `V`: Velocidad lineal
- `W`: Velocidad angular
- `pre`: Progreso del pre-avance (actual/total cm)
- `lane`: Estado de detección de línea (conf, offset)
- `reacq`: Si cumple criterios de re-adquisición

## Parámetros Ajustables

Todos los parámetros son configurables en tiempo real con `ros2 param set`:

```bash
# Pre-avance antes de girar (cm)
ros2 param set /line_follower commit_turn_pre_advance_cm 10.0

# Velocidad durante commit
ros2 param set /line_follower commit_speed 0.08

# Velocidad angular del giro
ros2 param set /line_follower commit_turn_w 0.6

# Duración máxima del commit (segundos)
ros2 param set /line_follower commit_duration 3.5
```

## Comportamiento Esperado

### Antes:
```
1. Detecta intersección
2. Inicia giro casi inmediatamente (4cm pre-avance)
3. Puede agarrar borde durante el giro (conf=0.5 era suficiente)
4. Termina prematuramente o en posición incorrecta
```

### Ahora:
```
1. Detecta intersección
2. Avanza 10cm en línea recta (mejor posicionamiento)
3. Inicia giro suave
4. Solo re-adquiere con línea clara (conf≥0.7, offset<0.6)
5. Termina en la línea correcta, bien posicionado
```

## Testing

Para probar las mejoras:

1. **Rebuild:**
   ```bash
   colcon build --packages-select puzzlebot_ros
   ```

2. **Observar logs:**
   - Verificar que `pre=10/10cm` antes de girar
   - Monitorear `lane=conf=X,off=Y` durante el giro
   - Confirmar que `reacq=True` solo con línea clara

3. **Ajustar si necesario:**
   - Si gira muy tarde: reducir `commit_turn_pre_advance_cm`
   - Si gira muy temprano: aumentar `commit_turn_pre_advance_cm`
   - Si no re-adquiere: revisar threshold de confianza (0.7)

## Notas

- **Straight commits** no cambiaron (funcionaban bien)
- **Turn commits** ahora son más conservadores y robustos
- Los parámetros se guardan automáticamente en `~/.ros/line_follower_params.json`
