# Fase 2 — transporte de pacotes

Escrito em 2026-10-09. Como a Fase 2 funciona neste stack, peça por peça, e o
que ainda falta. A estrutura geral (o que vai em cada pacote) está em
[`ARQUITETURA.md`](ARQUITETURA.md); as regras, em `REGRAS-CBR-2026.pdf` (p. 10-11).

## 1. A prova em uma frase

Levar 3 kits de primeiros socorros das 3 bases no topo do bloco da Fase 4
(1,5 m) para 3 bases vazias, **um por vez**, pousando com **o trem de pouso
inteiro dentro da base** para pegar e para entregar, e voltar sozinho para a
decolagem. 10 min por tentativa (no relógio do drone). Pousar fora de uma base
encerra a tentativa. Máximo 720 pontos (40 por pegar + 20 por entregar, ×2 pela
volta autônoma, ×2 open-hardware).

## 2. Como rodar

```bash
./scripts/docker_up.sh --phase2 --ground-truth          # sim, seed padrão
BASES_SEED=3 ./scripts/docker_up.sh --phase2 --debug    # outra arena + janelas
```

`--phase2` sobe `phase2_sim.launch.py` (sources do sim + `phase2.launch.py`) e
exporta `ARENA_PHASE=2`, que faz o ardubridge spawnar o layout da Fase 2.

## 3. As peças

```
 phase2_bases.yaml ──► arena.py ──► route.plan_deliveries ──► ordem dos 3 kits
                                                                   │
 phase2_mission_node (estratégia) ─────────────────────────────────┘
   ├─ Vehicle  ─────► controller_node ─► MAVROS        (armar, decolar, pousar)
   ├─ NavigateTo ───► nav_node (A* no octomap)         (pernas entre bases)
   ├─ VisualDescent ◄─ kit_detector / pad_detector_down (descer centralizado)
   └─ Gripper ──────► /hydrone/gripper/command
                        ├─ sim:  ardubridge_node  → world command "Gripper"
                        └─ real: gripper_dynamixel_node → AX-12A
```

| Arquivo | O que faz |
|---|---|
| `hydrone_bringup/config/phase2_bases.yaml` | **Onde estão as bases**, no frame da ARENA (Figura 7 das regras, grid 0,5 m). É o que se edita na prova, com as posições que a organização der. `kit_yaw_deg` gira o kit no sim |
| `hydrone_nav/arena.py` | Lê o layout e converte arena → `map`, ancorado na base de decolagem: `map = home + Rz(arena_yaw)·(p − takeoff)`. No sim `arena_yaw = −90°`; no real, medir no dia |
| `hydrone_nav/route.py` → `plan_deliveries` | Ordem coleta→entrega mais curta, testando as 3!×3! = 36 combinações |
| `hydrone_mission/phase2_mission_node.py` | A máquina de estados (abaixo) |
| `hydrone_msgs/action/Gripper.action` | A interface da garra, igual no sim e no real |
| `hydrone_controller/gripper.py` | Cliente `Gripper.close()/open()` → Job, como o `Vehicle` |
| `biguasim_main` (ardubridge + interface.py) | Sim: spawna bases e kits (`ARENA_PHASE=2`) e serve a ação da garra |
| `hydrone_bringup/gripper_dynamixel_node.py` | Real: AX-12A (Protocol 1.0, U2D2). `sources_real.launch.py gripper:=true` |
| `hydrone_vision/models/lipo_seg_yolo11.pt` | YOLO do kit (detecção, classe `lipo`) |

## 4. A máquina de estados

```
WAIT → ARMING → TAKEOFF ─┐
   ┌─────────────────────┘
   ▼
 GOTO coleta ─► DESCEND (no KIT) ─► TOUCH ─► LAND ─► ACT (fecha a garra)
   ─► ARMING/TAKEOFF ─► [pegou? senão tenta de novo / pula o kit]
   ─► GOTO entrega ─► DESCEND (na BASE) ─► LAND ─► ACT (abre a garra)
   ─► ARMING/TAKEOFF ─► próximo kit
 depois do último: GOTO casa ─► LAND ─► DONE
```

| Estado | O que faz | Por quê |
|---|---|---|
| `GOTO` | `NavigateTo` até a base na altura de cruzeiro (2,5 m) | o A* desvia do bloco da Fase 4 |
| `DESCEND` | `visual_descent`: centraliza na detecção mais próxima do centro e desce até 1,2 m acima | o mesmo pouso da Fase 1 (36/36). Na coleta segue o KIT, na entrega a BASE |
| `TOUCH` | só na coleta: guarda onde o kit está, desce em **controle de posição** segurando esse x,y até o rangefinder marcar 0,22 m, fecha a garra | o LAND do ArduPilot desce sem correção e escorregava 15-20 cm: o drone tocava AO LADO do kit (medido 2026-10-09). É como o POC da garra faz |
| `LAND` | `vehicle.land()` até as hélices pararem | regra: trem de pouso dentro da base |
| `ACT` | fecha (coleta) ou abre (entrega) a garra | |
| confirmação | `pick_check:=gripper` (padrão): o `holding` da garra. `vision`: olha de cima se o kit saiu da base | ver §6 |

**Sem giros**: o heading do arme é mantido o voo inteiro — é girando que a
odometria visual se perde. Quando a YOLO souber a orientação do kit, alinhar
com um kit virado vira uma opção.

**Relógio**: o orçamento de 600 s conta o tempo do drone (`/clock` no sim) e
volta para casa com `return_reserve_s` de folga. O log `TASK:` registra cada
pega/entrega com o relógio — é o log da tarefa que a regra pede.

## 5. Opções (argumentos de `phase2.launch.py` / parâmetros do nó)

| Opção | Padrão | Faz |
|---|---|---|
| `layout_file` | `config/phase2_bases.yaml` | onde estão as bases |
| `arena_yaw_deg` | `-90` | rotação arena → map (sim). Medir no drone real |
| `cruise_alt` | `2.5` | altura das pernas |
| `pickup_touch` | `true` | descer até encostar no kit em controle de posição |
| `touch_range_m` | `0.22` | altura (rangefinder) em que a garra fecha |
| `pick_check` | `gripper` | como confirmar a pega (`gripper` / `vision`) |
| `pick_retries` | `1` | tentativas extras antes de pular um kit |
| `visual_descend_to_m` | `1.2` | até onde a descida visual vai antes do TOUCH/LAND |
| `gripper_reach_cm` (ardubridge) | `[0, 0, -12]` | Reach Offset do volume de pega no sim |

## 6. O que foi medido, e o que falta

**Funciona no sim — missão completa (seed 1, kits girados 90°, 2026-10-09):**

| evento | relógio da missão |
|---|---|
| kit 1 pego / entregue em A | 12 s / 43 s |
| kit 3 pego / entregue em C | 67 s / 87 s |
| kit 2 pego / entregue em B | 107 s / 137 s |
| pouso na decolagem | **158 s de 600** |

O log da engine confirma 3 `picked up` e 3 `released`, um por kit.

**Ponto a melhorar:** na entrega A a câmera perdeu a base a 1,8 m (o kit
pendurado tapa parte da vista) e o drone pousou na posição conhecida do layout
— a 14 cm do centro real, dentro da base. Funciona porque as posições são
conhecidas, mas depende do layout medido com precisão.

**Problemas conhecidos:**

- **A YOLO do kit vê reflexo como kit.** No dataset simulado: recall 0,36,
  precisão 0,38, falso positivo em 25 de 70 imagens sem kit. É por isso que a
  confirmação por visão (`pick_check:=vision`) não é o padrão. Dataset para
  retreinar: `~/Documents/kit_dataset_sim` (gerado por
  `scripts/kit_dataset/`).
- **No sim a garra não tem sensor**: `holding` repete o comando. Para saber de
  verdade se pegou, olhe o log da engine:
  `~/.local/share/biguasim/1.0.0/worlds/Competition/Linux/Biguasim/Saved/Logs/HolodeckLog.txt`
  (linhas `Gripper:` e `picked up`).
- **AX-12A sem calibrar**: posições aberta/fechada, torque e limite de carga
  do `gripper_dynamixel_node` são provisórios.
- **`arena_yaw_deg` e o layout no real**: medir no dia.
- **Orientação do kit**: o drone não gira para alinhar; depende da YOLO com
  orientação.

## 7. Engine (BiguaSim)

- O `SpawnMesh` aceita rotação: `[x, y, z, roll, pitch, yaw]` (graus, frame UE)
  — mudança em `bs_engine-drone-competition/.../CustomCommand.cpp`.
- A garra: `CustomCommand ["Gripper", "uav0-id0"] [fechar, rx, ry, rz]`; o nome
  do agente leva o sufixo `-id0`.
- `set_day_time` / `set_weather` **travam** este mundo — não use.
