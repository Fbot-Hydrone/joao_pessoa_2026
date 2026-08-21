# Rangefinder no simulador — como funciona (e por que é sim-only)

Como o rangefinder nadir (altímetro descendente) é injetado no ArduPilot **dentro
da simulação**, por que ele é um componente **SIM-ONLY**, e os dois bugs que
resolvemos pra ele funcionar de ponta a ponta (ROS → MAVLink → FCU).

> Contexto maior: [`SENSOR-CONFIG.md`](SENSOR-CONFIG.md) (spec Fase 1&2, EKF3,
> montagem física do VL53L1X) e a separação sim/real em `hydrone_bringup.launch.py`.

---

## O papel do rangefinder e a fronteira sim/real

No **drone real** o rangefinder é um **VL53L1X ligado por I²C direto no Pixhawk**:
o ArduPilot lê nativo e o dado **sai** pelo MAVROS como `/mavros/distance_sensor/*`.
Não existe nenhum nó de ponte no real — o dado nasce dentro do MAVROS.

No **simulador** não há I²C nem VL53L1X. Precisamos **injetar** no SITL o mesmo que
o I²C injetaria no real. Isso é feito por um shim, o `rangefinder_bridge`, que é o
análogo do `zed_mimic` (o mesmo padrão: um nó sim-only que produz um contrato que,
no real, vem do hardware).

```
   SIM (injeção ROS→FCU, sim-only)                         REAL (nativo)
BiguaSim RangeFinderSensor (LaserScan, nadir)          VL53L1X ── I²C ──► Pixhawk
        │                                                              │
   rangefinder_bridge  (LaserScan → Range)                    (ArduPilot lê nativo)
        │  publica em /mavros/rangefinder                              │
   MAVROS distance_sensor (modo SUBSCRIBER)               MAVROS distance_sensor
        │  envia DISTANCE_SENSOR ao FCU                    (modo PUBLISHER, FCU→ROS)
        ▼                                                              ▼
   ArduPilot RNGFND1                                        /mavros/distance_sensor/*
```

**Consequência:** o `rangefinder_bridge` **e** a config `mavros_distance_sensor.yaml`
em modo *subscriber* vivem **só** na camada de sources do sim (`sources_sim.launch.py`),
**nunca** no `sources_real.launch.py`. No real o mesmo plugin roda em modo *publisher*
sem essa yaml.

---

## O caminho de dados no sim (passo a passo)

1. **BiguaSim `RangeFinderSensor`** (`config.yaml`) — feixe nadir, `LaserCount: 1`,
   `rotation: [0, -90, 0]` (pitch −90 pra apontar pra baixo), `LaserMaxDistance: 40`.
   Publicado como `sensor_msgs/LaserScan` em `/biguasim/<agente>/RangeFinderSensor`.
2. **`rangefinder_bridge`** (`hydrone_bringup/rangefinder_bridge.py`) — assina o
   LaserScan, pega o retorno finito mais próximo dentro de `[min_range, max_range]`
   e publica um `sensor_msgs/Range` em **`/mavros/rangefinder`**.
3. **MAVROS distance_sensor (subscriber)** — escuta esse `Range` e envia
   `DISTANCE_SENSOR` ao FCU.
4. **ArduPilot `RNGFND1`** — recebe como rangefinder MAVLink (`RNGFND1_TYPE=10`,
   `RNGFND1_ORIENT=25` = down).

### Parâmetros do ArduPilot (em `config/params/holybro_sitl.parm`)
```
RNGFND1_TYPE    10     # MAVLink (dado injetado via MAVROS)  ← ÚNICO param que muda no real
RNGFND1_ORIENT  25     # PITCH_270 = pra baixo
RNGFND1_MIN_CM  20
RNGFND1_MAX_CM  4000
EK3_RNG_USE_HGT -1     # NÃO usar rangefinder como fonte de altura do EKF
```
O rangefinder é **referência de Z pra pouso/flare apenas** — fica **fora** da fusão
global de altura (Z do EKF continua no baro, `EK3_SRC1_POSZ=1`). O único param de
autopiloto que difere sim↔real é o `RNGFND1_TYPE` (sim `10` MAVLink; real `16`
VL53L1X I²C nativo). O resto do `.parm` é compartilhado.

---

## Os dois bugs que resolvemos (e por quê)

### Bug 1 — formato do yaml (config não chegava ao plugin)
`mavros_distance_sensor.yaml` estava no formato **MAVROS1** (mapa aninhado
`distance_sensor: { rangefinder: {...} }` sob `/**:`). Esta build (MAVROS2) espera
um **param único `config`** com uma **string YAML**, sob o nó `/**/distance_sensor`
(igual ao `apm_config.yaml` stock). No formato errado, nossa config era **ignorada**
e o plugin caía no default do stock (que subia em modo publisher). Correção:
```yaml
/**/distance_sensor:
  ros__parameters:
    config: |
      rangefinder:
        subscriber: true          # modo subscriber (ROS→FCU)
        id: 0
        orientation: PITCH_270
        ...
```
Carregado **depois** do `apm_config.yaml` no launch, então este `config` sobrescreve
o stock. **Sim-only** (no real o plugin publica, não assina).

### Bug 2 — mismatch de nome de tópico (`No Data` no QGround)
Com o plugin já em modo subscriber, o ArduPilot ainda acusava
`PreArm: Rangefinder 1: No Data`. Motivo: nesta build o subscriber escuta
`~/<nome>` a partir do namespace `/mavros`, ou seja **`/mavros/rangefinder`** — mas
o bridge publicava em `/mavros/distance_sensor/rangefinder`. `Subscription count: 0`
→ nada chegava ao FCU. Correção: alinhar a **saída do bridge** para
**`/mavros/rangefinder`** (via param `out_range` no `sources_sim.launch.py`, com o
default do bridge também corrigido). Sem tocar no plugin nem na yaml de config.

> **Atualização (2026-08-21).** O bridge voltou a publicar em
> `/mavros/distance_sensor/rangefinder` — mas **além** de `/mavros/rangefinder`,
> não no lugar dele. O elo com o FCU continua sendo `/mavros/rangefinder` (nada
> aqui mudou); a segunda publicação é um **mimic** do que o MAVROS publica no
> drone real, para que a camada de autonomia (`pad_map_node`) leia o mesmo nome
> de tópico na simulação e no hardware, sem `range_topic:=...`. Se o
> `Subscription count` de `/mavros/rangefinder` voltar a zero, o problema é o
> plugin — não é este mimic.

> Lição: `topic echo` no lado ROS **não** prova o elo — o que quebrava era
> ROS→MAVLink→FCU. O teste real é o `No Data` sumir no QGround e o
> `Subscription count` do tópico do bridge virar 1.

---

## Verificação (após `docker_up.sh` + rebuild)
1. Sensor do sim: `ros2 topic echo /biguasim/uav0_id0/RangeFinderSensor` → LaserScan
   com range plausível pra baixo (≈ altitude). Se vier "horizontal", ajustar
   `rotation`/socket em `config.yaml` (o RangeFinderSensor é nativamente horizontal).
2. Adapter: `ros2 topic echo /mavros/rangefinder` → um `Range`.
3. Elo ROS→FCU: `ros2 node info /mavros/distance_sensor` mostra `/mavros/rangefinder`
   com o bridge como publisher; `ros2 topic info /mavros/rangefinder --verbose` →
   `Subscription count: 1`. No QGround, `Rangefinder 1: No Data` some.
4. Z ainda no baro: `EK3_SRC1_POSZ=1` e `EK3_RNG_USE_HGT=-1` (não funde no EKF).

## Arquivos
- `src/biguasim-ros2/biguasim_main/config/config.yaml` — sensor `RangeFinderSensor` nadir.
- `src/hydrone_bringup/hydrone_bringup/rangefinder_bridge.py` — LaserScan → Range (SIM-ONLY).
- `src/hydrone_bringup/config/mavros_distance_sensor.yaml` — plugin em modo subscriber (SIM-ONLY).
- `src/hydrone_bringup/config/params/holybro_sitl.parm` — `RNGFND1_*`, `EK3_RNG_USE_HGT=-1`.
- `src/hydrone_bringup/launch/sources_sim.launch.py` — sobe o bridge + carrega a yaml.
