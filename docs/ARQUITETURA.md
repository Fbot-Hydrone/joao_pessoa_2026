# Arquitetura do Hydrone — a estrutura que DEVE ser seguida

Escrito em 2026-10-08. Vale para as quatro fases. Cada fase tem um dono; a
integração depois só funciona se todo mundo usar os mesmos blocos do mesmo
jeito. Se algo aqui atrapalhar o seu caso, mude ESTE documento junto com o
código — não contorne.

---

## 1. A ideia em uma figura

```
   SOURCES (sim ou real)            BLOCOS COMPARTILHADOS                 FASES
   ────────────────────            ─────────────────────                 ─────
   /zed/zed_node/*    ──┐
   /down_cam/*        ──┼──► hydrone_vision   pad_detector_node ──┐
   /mavros/*          ──┘    hydrone_map      pad_map, octomap  ──┼──► phase1_mission_node
                             hydrone_localization (odometria)    │    (phase2_..., phase3_...)
                                                                 │         │
                             hydrone_nav      nav_node + libs  ◄─┴─────────┤
                                                   │                       │
                             hydrone_controller  controller_node ◄─────────┘
                                                   │   (ações Arm/Takeoff/GoTo/Land)
                                                   ▼
                                                MAVROS ──► ArduPilot
```

**Três regras que não se quebram:**

1. **Só o `hydrone_controller` fala com o MAVROS para comandar.** Nenhuma
   fase chama `/mavros/cmd/*`, `/mavros/set_mode` nem publica em
   `/mavros/setpoint_*`. Ler `/mavros/state` e a pose é permitido.
2. **Uma fase é um nó de estratégia.** Ela decide O QUE fazer (qual base,
   quando pousar, quando voltar). O COMO (armar, decolar, desviar de
   obstáculo, detectar toque) vem dos blocos.
3. **Todo comportamento novo entra como OPÇÃO** (argumento de launch com
   padrão documentado), nunca substituindo o antigo em silêncio. É assim que
   se compara "com" e "sem" no sweep de seeds antes de decidir.

---

## 2. Os pacotes — o que vai em cada um

| Pacote | Responsabilidade | NÃO coloque aqui |
|---|---|---|
| `hydrone_msgs` | interfaces: msgs, srvs, **ações** | lógica |
| `hydrone_controller` | o único elo com o FCU | estratégia de fase |
| `hydrone_nav` | ir de A a B, cobrir uma área, pousar com precisão | o que fazer ao chegar |
| `hydrone_vision` | detectar coisas em imagem (bases, gestos, QR) | decisão de voo |
| `hydrone_map` | fundir detecções em mapa; octomap | quando visitar |
| `hydrone_localization` | onde o drone está | — |
| `hydrone_mission` | **as fases**: uma máquina de estados por fase | MAVROS, A*, OpenCV |
| `hydrone_bringup` | launches, parâmetros do ArduPilot, nós de sensor | lógica de missão |

Regra prática: se você está escrevendo `import cv2` dentro de `hydrone_mission`,
ou `CommandTOL` fora de `hydrone_controller`, está no pacote errado.

### Bibliotecas × nós

Cada pacote de bloco tem **bibliotecas** (módulos Python, de preferência sem
ROS, testáveis) e **nós** (a casca ROS em volta). Uma fase pode usar o bloco
de dois jeitos:

- **pelo nó** (ação/tópico) — desacoplado, é o jeito da integração;
- **pela biblioteca** em processo — quando a fase precisa de controle fino
  dentro do próprio tick (a fase 1 usa `navigator` e `precision_landing`
  assim, porque persegue a base enquanto voa).

---

## 3. Os blocos, um a um

### 3.1 `hydrone_controller` — voar

| Arquivo | O que é |
|---|---|
| `controller_node.py` | o nó. Ações `Arm`, `Takeoff`, `GoTo`, `Land` em `/hydrone/controller/*`; tópico `cmd_pose` para setpoint contínuo. Mantém os serviços antigos de junho (`arm`, `disarm`, `takeoff`, `land`) |
| `fcu.py` | todo o encanamento MAVROS: estado, comandos, stream de setpoint, motivo da recusa do FCU (`Arm:`/`PreArm:`) |
| `touchdown.py` | detector de toque no chão, sem ROS: parado por `land_settle_s` **e** desceu `min_descent_m` |
| `vehicle.py` | **o que a fase usa.** Cliente estilo Nectar |

Como uma fase voa (padrão para TODAS):

```python
from hydrone_controller.vehicle import Vehicle

class MinhaFase(Node):
    def __init__(self):
        ...
        self.vehicle = Vehicle(self)
        self._job = None
        self.create_timer(0.1, self._tick)

    def _do_decolar(self):                     # chamado pelo tick
        if self._job is None:
            self._job = self.vehicle.takeoff(2.5)   # retorna NA HORA
        elif self._job.done:
            if self._job.ok:
                self._enter(self.PROXIMO)
            else:
                self.get_logger().warn(self._job.message)
                self._job = None                     # tenta de novo
```

- `vehicle.arm()` → GUIDED + armar (repete por tempo, não por ack)
- `vehicle.takeoff(alt, hold_z)` → sobe `alt` acima de onde está pousado
- `vehicle.go_to(x, y, z, yaw)` → reta, espera chegar
- `vehicle.land(disarm=True)` → LAND, detecta toque, **para as hélices**
- `vehicle.setpoint(x, y, z, yaw)` → alvo contínuo, sem esperar (pairar, ajustes)

**Nunca bloqueie o tick esperando um Job.** Consulte `.done` a cada tick.

### 3.2 `hydrone_nav` — ir, cobrir, pousar

| Arquivo | O que é |
|---|---|
| `nav_node.py` | ação `/hydrone/nav/navigate_to`: planeja pelo octomap e voa os waypoints pelo `GoTo` do controller |
| `navigator.py` | `Navigator.plan(aqui, alvo, yaw) -> Leg`: reta livre, desvio por A*, ou **recusa** (`blocked`) |
| `planner.py` | A* 3D + simplificação do caminho |
| `coverage.py` | perímetro e faixas de varredura (largura calculada pela câmera) |
| `route.py` | quais bases valem a viagem (`is_candidate`), qual a mais próxima |
| `precision_landing.py` | a geometria de "estou em cima da base?" e as opções de pouso |
| `servo.py` | servo visual que aprende o mapeamento pixel→metro (opcional) |
| `nav_node_legacy.py` | o nav de junho, só para o stack antigo |

### 3.3 `hydrone_vision` — ver

| Arquivo | O que é |
|---|---|
| `pad_detector_node.py` | detecta bases e projeta no mundo. `detector_backend:=yolo` (padrão) ou `cv` |
| `yolo_pad_detector.py` / `pad_detector.py` | os dois backends, mesma interface `detect(img) -> [det]` |
| `vision_node.py` | ponto de partida das fases 3 (gestos) e 4 (QR) |

Novo detector = novo backend com a mesma interface + um valor novo em
`detector_backend`. Não crie um segundo nó de detecção de bases.

### 3.4 `hydrone_map` — lembrar

`pad_map_node` funde as detecções em `/hydrone/pads/map` (uma entrada por
base, com `observations`, altura medida, `visited`). Fusão por
`merge_radius` (detecção→entrada) e `dedupe_radius` (entrada→entrada).
`octomap_server` + `cloud_filter_node` mantêm o mapa 3D de ocupação.

---

## 4. A fase 1 (`hydrone_mission/phase1_mission_node.py`)

Máquina de estados, um handler `_do_<estado>` por estado, chamada pelo
`_tick` a 10 Hz:

```
WAIT_FCU → ARMING → REGISTER → TAKEOFF → SELECT ⇄ SETTLE ⇄ TRAVEL → CONFIRM → LAND → DWELL ─┐
               ▲                            │                                                │
               └────────────────────────────┴──────── (próxima base) ◄──────────────────────┘
                                         SELECT com a cota cumprida → volta para casa → DONE
```

| Estado | Faz | Usa |
|---|---|---|
| `WAIT_FCU` | espera MAVROS, pose e o controller | `Vehicle.ready()` |
| `ARMING` | arma | `vehicle.arm()` |
| `REGISTER` | diz ao mapa "esta é a base de decolagem" | serviço do `pad_map` |
| `TAKEOFF` | sobe; 3 recusas → `ABORTED` | `vehicle.takeoff()` |
| `SELECT` | escolhe: base conhecida, próxima perna da busca, ou casa | `route` |
| `SETTLE` | espera parar antes de olhar/girar | velocidade do EKF |
| `TRAVEL` | voa a perna; persegue a base se ela mudar no mapa | `navigator`, `setpoint` |
| `CONFIRM` | paira e confirma pela câmera de baixo | `precision_landing` |
| `LAND` | pousa e para as hélices | `vehicle.land()` |
| `DWELL` | fica parado na base, conta o pouso, decide o próximo | serviço `mark_visited` |

### As opções da fase 1 (argumentos de `phase1.launch.py`)

| Opção | Padrão | Faz |
|---|---|---|
| `down_detector_backend` / `forward_detector_backend` | `yolo` | `yolo` ou `cv` |
| `centre_on_pad` | `false` | servo visual antes de pousar |
| `land_centre_max_cm` | `0` (desl.) | veta pouso com a base fora do centro |
| `trust_map_observations` | `20` | sem confirmação pela câmera mas com mapa forte → pousa no mapa |
| `confirm_detections` / `confirm_confidence` | `6` / `0.30` | quantas olhadas e com que confiança |
| `target_bases`, `takeoff_alt`, `mission_budget_s` | `6`, `2.5`, `600` | a prova |
| `dry_run` | `false` | ensaio com o drone na mão: nada comanda o veículo |

Ligar algo na linha de comando:

```bash
./scripts/docker_up.sh --phase1 --debug --ground-truth centre_on_pad:=true
```

---

## 5. Como fazer coisas comuns

**Uma fase nova** — crie `hydrone_mission/phaseN_mission_node.py` copiando o
esqueleto de estados da fase 1 (`_enter`, `_tick`, `_do_*`, `_job`), registre
o executável em `hydrone_mission/setup.py` e um `phaseN.launch.py` em
`hydrone_bringup`. Voe só pelo `Vehicle`; navegue pelo `nav_node` ou pelo
`Navigator`.

**Uma opção nova** — `declare_parameter` no nó com o padrão ANTIGO,
`DeclareLaunchArgument` no launch com descrição, e só depois de um sweep de
seeds mudar o padrão. Escreva no comentário o número medido que justificou.

**Uma interface nova** — arquivo em `hydrone_msgs/{msg,srv,action}` e linha
no `CMakeLists.txt`. Precisa de `docker compose build` (o `--dev` não gera
interfaces).

**Validar uma mudança de comportamento** — testes do pacote, depois
`scripts/seed_sweep.sh 1 2 3 4 5 6` e `scripts/score_run.py`. Um voo numa
seed não prova nada.

---

## 6. Testes

```bash
docker run --rm -v $PWD/src:/ws/src_host:ro joao_pessoa_2026-hydrone:latest bash -lc '
  source /opt/ros/humble/setup.bash; source /ws/install/setup.bash
  export PYTHONPATH=/ws/src_host/hydrone_map:/ws/src_host/hydrone_nav:/ws/src_host/hydrone_controller:/ws/src_host/hydrone_mission:$PYTHONPATH
  for d in hydrone_mission hydrone_controller hydrone_nav hydrone_map; do
    cd /ws/src_host/$d && python3 -m pytest -q test; done'
```

Todos verdes em 2026-10-08. Um teste que falha é um bug ou um teste velho —
nunca deixe falhando "porque sempre falhou".
