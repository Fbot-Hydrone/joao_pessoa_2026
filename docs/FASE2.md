# Fase 2 — transporte de pacotes

Escrito em 2026-10-09, revisto em 2026-11-23 (branch `cbr/phase2`). Como a
Fase 2 funciona neste stack, peça por peça, e o que ainda falta. A estrutura geral (o que vai em cada pacote) está em
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
`PHASE2_LAYOUT=<yaml>` spawna de outro arquivo (ex.: kits girados).

No drone (Orin):

```bash
ros2 launch hydrone_bringup phase2_real.launch.py \
    arena_walls_m:="[frente, tras, esquerda, direita]" \
    pickup_1:="[x, y]" pickup_2:="[x, y]" pickup_3:="[x, y]" \
    delivery_1:="[x, y]" delivery_2:="[x, y]" delivery_3:="[x, y]"
```

`phase2_real` = `sources_real` (com `gripper:=true`) + `phase2.launch.py`. As
posições são medidas **a partir da base de decolagem**, x para a direita e y
para onde o nariz aponta ao armar (`config/phase2_bases_real.yaml`,
`arena_yaw_deg 0`). Uma base sem posição (ou `:=scan`) faz o drone varrer a
arena antes dos kits (§4). Também aceita `{grid: [col, lin]}`. Os argumentos
de `sources_real` (`surface_offset`, `takeoff_base_height_m`, ...) são os
mesmos da Fase 1.

Na arena o drone NÃO roda `phase2_real` à mão: `orin_flight/run_phase2.sh`
(como o da Fase 1) sobe `sources_real gripper:=true`, mede onde está o chão no
z da FCU (`floor_z.py`), sobe `phase2.launch.py` com `ground_z` e a caixa
deslocados por ele, `auto_start:=false`, e faz a contagem e o
`/hydrone/mission/start`. O código vai para a Orin numa worktree separada
(`deploy_phase2.sh`). Run sheet: `orin_flight/PHASE2.md`.

Cada execução grava `log_dir/phase2_<data>/` (padrão `~/hydrone_logs`, no sim
`/ws/logs` = `./logs`): `tasks.log` (o log de tarefas da regra) e `bag/`
(`record:=false` desliga, `record_images:=true` inclui a câmera de barriga,
~8 GB em 10 min).

## 3. As peças

```
 pickup_N / delivery_N ─┐
 phase2_bases.yaml ─────┴─► arena.py ─┬─► route.plan_deliveries ──► ordem dos kits
 (alguma desconhecida) ─► varredura ──┘ (pad_map + kits vistos → phase2_core.fill_missing)
                                                                   │
 phase2_mission_node (estratégia) ─────────────────────────────────┘
   ├─ Vehicle  ─────► controller_node ─► MAVROS        (armar, decolar, pousar)
   ├─ NavigateTo ───► nav_node (A* no octomap)         (pernas entre bases)
   ├─ VisualDescent ◄─ kit_detector / pad_detector_down (descer centralizado)
   ├─ KitAxis ◄────── kit_axis_node                    (para onde o kit aponta)
   ├─ /mavros/state                                    (o piloto mexeu? acaba)
   └─ Gripper ──────► /hydrone/gripper/command
                        ├─ sim:  ardubridge_node  → world command "Gripper"
                        └─ real: gripper_dynamixel_node → AX-12A
```

| Arquivo | O que faz |
|---|---|
| `hydrone_bringup/config/phase2_bases.yaml` | Layout do SIM, no frame da ARENA (Figura 7 das regras, grid 0,5 m). `kit_yaw_deg` gira o kit no sim. No drone: `phase2_bases_real.yaml` (vazio) + os argumentos por base |
| `hydrone_nav/arena.py` | Lê o layout e converte arena → `map`, ancorado na base de decolagem: `map = home + Rz(arena_yaw)·(p − takeoff)`. No sim `arena_yaw = −90°`; no real, medir no dia |
| `hydrone_nav/route.py` → `plan_deliveries` | Ordem coleta→entrega mais curta, testando as 3!×3! = 36 combinações |
| `hydrone_mission/phase2_mission_node.py` | A máquina de estados (abaixo) |
| `hydrone_mission/phase2_core.py` | As decisões sem ROS, testadas (`test_phase2_core.py`): posições por parâmetro, a varredura, quem é coleta/entrega, o limite do toque, o giro para o kit, a trava do piloto |
| `hydrone_vision/kit_axis.py` + `kit_axis_node.py` | Eixo longo do kit na imagem (`/hydrone/kits/axis`, `KitAxis.msg`). Roda com qualquer câmera + detector que publique `PadDetection` |
| `hydrone_bringup/launch/phase2_real.launch.py` | O drone: `sources_real` + `phase2.launch.py`, só com fatos do drone (offset da câmera, `ground_z`, frame) |
| `hydrone_msgs/action/Gripper.action` | A interface da garra, igual no sim e no real |
| `hydrone_controller/gripper.py` | Cliente `Gripper.close()/open()` → Job, como o `Vehicle` |
| `biguasim_main` (ardubridge + interface.py) | Sim: spawna bases e kits (`ARENA_PHASE=2`) e serve a ação da garra |
| `hydrone_bringup/gripper_dynamixel_node.py` | Real: AX-12A pela ArbotiX-M. Faz homing por torque no boot (garra VAZIA), fecha com rampa de torque e só diz HOLDING quando a garra para e a carga sobe, segura com torque limitado. `config/gripper.yaml`, `sources_real.launch.py gripper:=true`. Detalhes: [`GRIPPER.md`](GRIPPER.md) |
| `hydrone_vision/models/lipo_mix_yolo11s.pt` | YOLO do kit (detecção, classe `lipo`): yolo11s treinado com o sim + 128 fotos reais da C270 (2026-10-27). Dataset em `~/Documents/kit_dataset_mix` |

## 4. A máquina de estados

```
WAIT → ARMING → TAKEOFF ─┬─ (base desconhecida) SCAN ─┐
   ┌─────────────────────┴────────────────────────────┘
   ▼
 GOTO coleta ─► DESCEND (no KIT) ─► ALIGN ─► TURN ─► DESCEND ─► TOUCH ─► LAND
   ─► ACT (fecha a garra) ─► ARMING/TAKEOFF ─► [pegou? senão tenta de novo / pula]
   ─► TURN (volta ao heading do arme) ─► GOTO entrega ─► DESCEND (na BASE)
   ─► LAND ─► ACT (abre a garra) ─► ARMING/TAKEOFF ─► próximo kit
 depois do último: GOTO casa ─► LAND ─► DONE
 a qualquer hora: arme/desarme/modo que a missão não pediu ─► ABORTED (não rearma)
```

| Estado | O que faz | Por quê |
|---|---|---|
| `SCAN` | só se alguma base não foi dada: o `map_sweep` da Fase 1 (perímetro fechado + faixas espaçadas pela pegada da câmera de barriga), com o `pad_map` juntando as bases e a câmera de barriga contando kits. No fim, base com kit (≥ 3 vistas) = coleta, sem kit = entrega; faltando kit, as bases altas (≥ 1 m) completam as coletas | as regras só garantem as posições das entregas. As faixas não giram 180° no fim como na Fase 1: a câmera de barriga vê a mesma faixa |
| `GOTO` | `NavigateTo` até a base na altura de cruzeiro (`cruise_alt` acima do CHÃO: 3,2 m no sim, 3,0 no real) | o A* desvia do bloco da Fase 4 |
| `DESCEND` | `visual_descent`: centraliza na detecção mais próxima do centro e desce até 1,2 m acima | o mesmo pouso da Fase 1 (36/36). Na coleta segue o KIT, na entrega a BASE |
| `ALIGN` / `TURN` | parado sobre o kit, lê o eixo do kit (`kit_axis`, 3 leituras em até 2 s) e gira até o kit ficar frente-trás (`gripper_axis_deg 0`: os dedos fecham pelos lados). Sem leitura coerente, desce no heading atual. Depois de pegar, gira de volta ao heading do arme ANTES de andar | a regra deixa o kit em qualquer orientação, e a garra só pega pelo lado de 8 cm (antiga ~±8°, cremalheira ~±30°). Girar e andar nunca juntos |
| `TOUCH` | só na coleta: guarda onde o kit está, desce em **controle de posição** segurando esse x,y até o rangefinder marcar 0,22 m, fecha a garra. O ponto fica a no máximo `touch_max_off_m` (0,22) do centro da base, visto pela YOLO da base na descida (senão a posição conhecida); se teria que puxar mais de 5 cm, **pula o kit** | o LAND do ArduPilot desce sem correção e escorregava 15-20 cm (medido 2026-10-09). E o trem inteiro tem que ficar na base de 1 m: o pé mais longe está a 0,22 m do centro (CAD v3), 0,5 − 0,22 − 0,06 de margem = 0,22 |
| `LAND` | `vehicle.land()` até as hélices pararem | regra: trem de pouso dentro da base |
| `ACT` | fecha (coleta) ou abre (entrega) a garra. Fecha UMA vez, no chão (`close_in_touch:=false`) | no ar, a 0,22 m, os dedos reais ficam 1-2,5 cm acima do kit |
| `OPENING` | abre a garra e ESPERA: antes de armar, e pairando antes de tentar de novo ou pular um kit | garra fechada não pode descer de novo sobre o kit |
| confirmação | `pick_check:=gripper` (padrão): o `holding` da garra. `vision`: olha de cima se o kit saiu da base | ver §6 |
| kit perdido | na coleta, sobe e olha de novo 2 vezes; depois **pula o kit** | pousar às cegas no centro pode pôr um pé no kit, a 1,5 m do chão (decisão do Ivo, 2026-11-23) |

**Um heading só**: o do arme, menos sobre o kit, onde o drone gira parado e
volta antes da próxima perna. Translação e rotação nunca juntas.

**O piloto**: o RC só desliga o drone, e na Fase 2 um humano pode mandá-lo para
casa. Por isso qualquer desarme em voo, modo ≠ GUIDED em voo, ou arme/troca de
modo com o drone parado numa base que a missão não pediu vira `ABORTED`, e a
missão nunca mais arma. Também aborta: parado > 12° inclinado numa base (um pé
no kit ou na borda; decolar dali vira o drone) e FCU recusando rearmar por 30 s
(kill switch ativo: soltá-lo não pode decolar o drone na mão de alguém).

**Relógio**: o orçamento de 600 s conta o tempo do drone (`/clock` no sim) e
volta para casa com `return_reserve_s` de folga. O log `TASK:` registra cada
pega/entrega com o relógio, no terminal e em `tasks.log` — é o log da tarefa
que a regra pede. A varredura conta no relógio.

## 5. Opções (argumentos de `phase2.launch.py` / parâmetros do nó)

| Opção | Padrão | Faz |
|---|---|---|
| `layout_file` | `config/phase2_bases.yaml` | onde estão as bases (real: `phase2_bases_real.yaml`) |
| `pickup_1..3`, `delivery_1..3` | `""` | uma base: `""` = a do layout, `scan` = desconhecida, `"[x, y]"` ou `"{grid: [c, l]}"` |
| `arena_yaw_deg` | `-90` | rotação arena → map (sim). Real: `0` |
| `ground_z` | `-0.7` | o chão no z da FCU (real: o `ground_z()` da Fase 1) |
| `cruise_alt` | `3.2` | altura das pernas, m acima do CHÃO (real: `3.0`) |
| `arena_walls_m` | `[0.0]` | `[frente, trás, esq, dir]` como na Fase 1: caixa do A* e da varredura no real |
| `align_kit` / `gripper_axis_deg` | `true` / `0` | girar o kit para a garra / em que direção do corpo ele tem que ficar |
| `touch_max_off_m` | `0.22` | máx. do centro do drone ao centro da base de coleta |
| `record` / `record_images` / `log_dir` | `true` / `false` / `~/hydrone_logs` | rosbag + log de tarefas |
| `auto_start` | `true` | `false`: espera `/hydrone/mission/start` (a contagem da arena) |
| `close_in_touch` | `false` | fechar no ar no TOUCH e reaproveitar no ACT (versão da Orin de 24/11) |
| `yolo_device` | `cpu` | `0` = CUDA (o `run_phase2.sh` usa `0`) |
| `pickup_touch` | `true` | descer até encostar no kit em controle de posição |
| `touch_range_m` | `0.22` | altura (rangefinder) em que a garra fecha |
| `pick_check` | `gripper` | como confirmar a pega (`gripper` / `vision`) |
| `pick_retries` | `1` | tentativas extras antes de pular um kit |
| `visual_descend_to_m` | `1.2` | até onde a descida visual vai antes do TOUCH/LAND |
| `gripper_reach_cm` (ardubridge) | `[0, 0, -12]` | Reach Offset do volume de pega no sim |

## 6. O que foi medido, e o que falta

**`cbr/phase2` no sim (máquina 2, 2026-11-23, seed 1, odometria = ground
truth, com viewport):**

| run | o que testa | resultado |
|---|---|---|
| 4 | kits girados (125°, 30°, 0° no frame da arena), todas as posições dadas | 3/3 entregues, casa em **197 s**. Kit 1 virou certo (eixo 51° → −84° na imagem). Kit 3 sem leitura (janela de 2 s de parede = 0,7 s de sim) e kit 2 leu errado com o drone ainda assentando: pegou na 2ª tentativa |
| 5 | o mesmo + `pickup_2:=scan delivery_3:=scan`, com as correções do giro | varredura achou as duas a ~14 cm (coleta 2 com 103 vistas de kit, entrega 3 com 0, topo 1,08 m). Giros −85°/+35°/−69° e uma correção de 8° (2ª passada). **3/3, casa em 366 s**, dos quais 166 s de varredura |
| 7b | o mesmo, com a parada antecipada da varredura | parou no setpoint 16 de 20 (**134 s**), mesmas posições. Giros +34°/−79°/−57° (+5° na 2ª passada). **3/3, casa em 341 s** |

A varredura só para cedo com evidência clara: coleta = base com ≥ 3 vistas de
kit; entrega = base vista ≥ 6 vezes, nenhum kit, e fora da altura do bloco (uma
base na altura das coletas sem kit visto ainda pode ser coleta). Sem isso, vai
até o fim e classifica com o que viu.

Os tempos de giro e da leitura do eixo contam no relógio do VEÍCULO (`/clock`
no sim, que roda a ~0,35×). Sem viewport o BiguaSim renderiza as câmeras quase
pretas: a missão não vê kit e pula os três (o que é o comportamento certo para
uma câmera cega).

O bag de um run morto (sem Ctrl-C) fica sem `metadata.yaml`: `ros2 bag
reindex <pasta>` refaz. No drone, terminar o launch com Ctrl-C.

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
- **Garra**: o homing acha as posições sozinho no boot; os torques de
  `gripper.yaml` vieram da bancada da Orin, nunca de um voo.
- **Posições no real**: perguntar à organização em que frame vêm. O
  `phase2_bases_real.yaml` mede a partir da base de decolagem; se vierem no
  frame da arena, pôr a base de decolagem em `takeoff` e o `arena_yaw_deg`.
  Os parâmetros travam depois da 1ª tentativa.
- **Kit perdido em voo** (cai da garra): o log ainda diz DELIVERED; o drone
  não sabe.
- **Base de coleta girada**: o limite do toque é um círculo (vale em qualquer
  giro). Se as bases forem quadradas e alinhadas ao drone, dá para soltar
  `touch_max_off_m` até ~0,3.
- **`kit_axis` no real**: testado em 66 fotos reais da C922 (base do lab com
  linhas amarelas claras): 60 coerentes e certas no olho, as 6 restantes
  borradas (= não gira). Nunca em voo.

## 7. Engine (BiguaSim)

- O `SpawnMesh` aceita rotação: `[x, y, z, roll, pitch, yaw]` (graus, frame UE)
  — mudança em `bs_engine-drone-competition/.../CustomCommand.cpp`.
- A garra: `CustomCommand ["Gripper", "uav0-id0"] [fechar, rx, ry, rz]`; o nome
  do agente leva o sufixo `-id0`.
- `set_day_time` / `set_weather` **travam** este mundo — não use.
