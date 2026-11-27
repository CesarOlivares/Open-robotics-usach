# Especificación de diseño del hexápodo (E8)

Cotas para diseñar y fabricar las piezas del robot real, obtenidas con el
estudio de dimensiones (`analysis/dimension_study.py`) a partir de los
componentes que ya tienes. Todas se verificaron en simulación.

> Las cotas son **de eje a eje de giro** (no de borde a borde de la pieza).
> La pieza puede tener cualquier forma mientras respete esas distancias, los
> espacios libres y la masa supuesta (§5).

## 1. Cotas principales

| Cota | Valor | Qué es | Por qué |
|---|---|---|---|
| **L1 coxa** | **45 mm** | eje de la coxa (vertical) → eje del fémur (horizontal) | mínimo 40,3 mm para que el servo del fémur no toque la placa; +4 mm para el rodamiento del adaptador coaxial |
| **L2 fémur** | **60 mm** | eje del fémur → eje de la rodilla | mínimo 45,7 mm (los 2 servos van uno tras otro). Centro de la región que cumple (§3) |
| **L3 tibia** | **115 mm** | eje de la rodilla → punta del pie | centro de la región que cumple |
| **R cuerpo** | **80 mm** | centro del cuerpo → eje de cada coxa | mínimo 79 mm para que la batería quepa bajo el cuerpo y las PCA9685 no toquen los servos |
| Pie | esfera Ø 20 mm | punta de la tibia | contacto en un punto; goma para fricción μ ≈ 1 |

Comparación con el diseño de partida (45 / 80 / 120 mm, R = 100 mm): el fémur
pasaba el torque de diseño en el extremo del paso (0,555 > 0,54 N·m) y la
batería quedaba a 52 mm del suelo (mín. 60). El diseño nuevo es **más
compacto** (pata 220 mm de largo total en vez de 245; cuerpo 20 mm más chico de radio) y cumple
todo.

## 2. Cuerpo

Hexágono **regular y horizontal**, dos placas iguales. El frente (+x) mira a
un **lado plano**; las patas salen de los 6 vértices.

| Cota | Valor |
|---|---|
| Radio al eje de la coxa R | 80 mm |
| Radio de la placa (centro → vértice) | 95 mm (la placa sobresale 15 mm del eje) |
| Lado del hexágono | 95 mm |
| Apotema (centro → lado) | 82,3 mm |
| Ancho de lado plano a lado plano | 164,5 mm |
| Separación entre placas (cara interior a cara interior) | 45 mm (servo de 42,9 mm de alto) |
| Espesor de placa (supuesto) | 3 mm |
| Ángulos de los ejes de coxa (desde +x, antihorario) | LF +30°, LM +90°, LR +150°, RF −30°, RM −90°, RR −150° |
| Ejes de coxa en el plano (x, y) [mm] | LF (69,3; 40,0) · LM (0; 80,0) · LR (−69,3; 40,0) · RF (69,3; −40,0) · RM (0; −80,0) · RR (−69,3; −40,0) |

**Servo de cada coxa:** entre las placas, eje de salida vertical **justo en
el vértice** (a 80 mm del centro), cuerpo del servo hacia el centro. El
centro del servo queda a 69,7 mm del centro del cuerpo y su extremo
interior a 49,3 mm.

## 3. Layout de componentes

Origen: centro del cuerpo, a media altura entre las placas. x adelante,
y izquierda, z arriba.

| Componente | Posición del centro (x, y, z) [mm] | Orientación | Montaje |
|---|---|---|---|
| Batería LiPo 3S 137 × 43 × 22 | (0; 0; −36,5) | largo en x | **bajo la placa inferior**, centrada. Entre placas no cabe (necesita 72 mm de radio; los servos dejan 49–70) |
| Raspberry Pi 4 (85 × 56) | (0; 0; +39) | largo en x | sobre la placa superior, separadores de 5 mm |
| PCA9685 #1 (0x40, patas izquierdas) | (0; +17,5; −15) | largo en x | sobre la placa inferior, entre placas |
| PCA9685 #2 (0x41, puente A0, patas derechas) | (0; −17,5; −15) | largo en x | ídem |
| IMU BNO085 | (0; 0; +20,5) | ejes alineados con el cuerpo | bajo la placa superior, **en el centro** |
| Cámara Pi Module 3 | (76,5; 0; +37,5) | mirando a +x, 10° hacia el suelo | sobre la placa superior, en el lado frontal |

Con este layout el centro de masa queda en el eje vertical del cuerpo
(< 5 mm, comprobado por test). Holguras: batería 9 mm dentro del borde de la
placa; PCA9685 a 1 mm del extremo interior de los servos de coxa (**ajustada:
fijar las PCA con separadores y revisar al montar**).

## 4. Postura y marcha de diseño

| Parámetro | Valor |
|---|---|
| Pie de pie: distancia horizontal eje del fémur → pie (d) | 60 mm |
| Pie de pie: altura del eje de la coxa sobre el centro del pie (H) | 100 mm |
| Altura del cuerpo (centro) sobre el suelo | 110 mm |
| Altura libre bajo la batería | **62,5 mm** (mín. pedido 60) |
| Paso máximo (S) | 60 mm → 0,10 m/s con T = 1 s, β = 0,6 |
| Altura de vuelo del pie (h) | 30 mm |
| Ángulos en la postura de pie | coxa 0°, fémur +14,5° (rodilla 15 mm sobre el eje del fémur), rodilla −103,5°: la tibia queda vertical (a 1°) |

## 5. Márgenes obtenidos

Torque de diseño = 50 % del de bloqueo del MG996R a 6 V (1,08 N·m) = 0,54 N·m.

| Restricción | Valor | Mínimo exigido | ✓ |
|---|---|---|---|
| Torque del fémur, peor punto del paso (3 patas, pie a 90 mm) | 0,506 N·m | ≤ 0,54 | ✅ FS 2,14 |
| Torque de la tibia, peor punto | 0,187 N·m | ≤ 0,54 | ✅ FS 5,8 |
| Alcance (sin estirar la pata más del 90 %) | 23 mm de holgura | ≥ 0 | ✅ |
| Distancia a los topes articulares | 0,25 rad | ≥ 0,10 | ✅ |
| Rodilla sobre el eje del fémur en todo el apoyo | 11 mm | ≥ 0 | ✅ |
| Altura libre | 62,5 mm | ≥ 60 | ✅ |
| Margen de estabilidad estático | 62 mm | ≥ 40 | ✅ |
| Distancia entre pies vecinos (peor caso) | 125 mm | ≥ 30 | ✅ |
| Holgura entre servos de fémur vecinos | 18 mm | ≥ 5 | ✅ |
| Batería cabe bajo el cuerpo | 9 mm | ≥ 0 | ✅ |
| PCA9685 no tocan los servos de coxa | 1 mm | ≥ 0 | ✅ (ajustado) |

**Validación del modelo de torque en Gazebo** (robot quieto en 3 patas):

| Punto | Modelo fémur / tibia | Gazebo fémur / tibia |
|---|---|---|
| Diseño de partida, pie a 70 mm | 0,374 / 0,025 | 0,375 / 0,025 |
| Diseño nuevo, pie a 60 mm (neutro) | 0,325 / 0,012 | 0,325 / 0,012 |
| Diseño nuevo, pie a 90 mm (peor punto) | 0,506 / 0,187 | 0,506 / 0,188 |

Caminando, Gazebo no sirve para medir carga (en modo posición las patas
apoyadas se empujan entre sí y el torque marca saturado; ver E4). Por eso
los márgenes se calculan con el modelo estático, validado arriba.

**Sensibilidad** (factor de seguridad del peor servo):

| Caso | FS |
|---|---|
| Diseño, 6 V | 2,14 |
| Servos a 4,8 V (bloqueo 0,92 N·m) | **1,82** |
| Robot 200 g más pesado | **1,91** |
| Piezas de la pata 50 % más pesadas (sin cambiar la masa total) | 2,17 |

## 6. Recomendaciones para el robot real

1. **Alimentar los servos a 6 V** (UBEC de 10–20 A desde la LiPo). A 4,8 V
   el margen baja a 1,82.
2. **Presupuesto de masa: 1,88 kg en total.** La estructura (placas, piezas de
   las patas) se supuso en 400 g. Si el robot real pesa más de ~2,1 kg, el
   fémur pierde el factor 2: acortar d (pies más cerca del cuerpo) o bajar
   el paso.
3. Piezas de la pata supuestas: soporte de coxa 8 g, soporte de fémur 10 g,
   tibia 15 g. Si salen distintas, cambiarlas en `hexapod_params.xacro` y
   volver a correr `analysis/dimension_study.py` y `colcon test`.
4. Probar cada MG996R antes de montar: las copias suelen dar menos torque que
   la hoja de datos.
5. Montar las PCA9685 con cuidado: es la holgura más ajustada (1 mm).

## 7. Cómo rehacer el estudio

```bash
source install/setup.bash
python3 analysis/dimension_study.py     # imprime pasos, elige y guarda figuras
colcon test                             # test_design_study: el modelo actual cumple todo
```

Figuras: `docs/img/estudio_torque_vs_d.png`, `estudio_mapa_largos.png`,
`estudio_pata.png`, `layout.png`. Resultados: `analysis/resultado_estudio.json`.
