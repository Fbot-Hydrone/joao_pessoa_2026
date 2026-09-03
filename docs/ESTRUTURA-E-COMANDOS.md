# A estrutura do stack, e o que cada comando sobe

Escrito em 2026-09-02. Este é o mapa: quais nós existem, quem sobe quem, e o
que cada comando de fato chama. Se você quer *rodar*, a seção 1 basta; se algo
não subiu, a 3 diz de quem era a responsabilidade.

---

## 1. Os comandos

Todos passam pelo mesmo script, que constrói a imagem, sobe o container e roda
**um** launch dentro dele:

```bash
./scripts/docker_up.sh <flags>
```

O script acha o repo do BiguaSim sozinho, procurando por ele ao lado deste
(`../biguasim-competicao/bs-drone-competition` e mais dois nomes comuns). Se o
seu estiver em outro lugar, `BS_SIM_DIR=/caminho/para/ele` sobrepõe.

Ele também **limpa antes de subir** o que a corrida anterior vazou: os
`.utrace` do Unreal Insights (profiling puro — um único voo longo deixou 87 GB,
e um mês deles levou o disco a 100%) e os segmentos `HOLODECK_MEM` do
`/dev/shm`.

### As missões

| comando | chama | o que é |
|---|---|---|
| `--phase1` | `phase1_sim.launch.py` | **a missão padrão.** ZED mapeia, barriga acha e posiciona |
| `--zed-detect` | `phase1_zed_detect_sim.launch.py` | a divisão antiga: ZED acha e posiciona, barriga só confirma |
| `--landing-sites` | `landing_sites_sim.launch.py` | a missão anterior a tudo isso: voa +X e pousa no que vir |
| *(nenhuma)* | `hydrone_sim.launch.py` | só os sensores e o SITL, **sem autonomia** — você pilota pelo QGC |

São mutuamente exclusivos; a última vence.

### Os modificadores

| flag | efeito |
|---|---|
| `--ground-truth` | o EKF voa na verdade do simulador em vez da odometria visual. **Ferramenta de depuração** — separa bug de autonomia de bug de localização, e nunca prova que algo funciona |
| `--debug` | abre **rviz2** com o layout da missão e **rqt_image_view** na visão anotada da barriga |
| `--dev` | monta os pacotes do host no container: editar nó, `docker compose restart hydrone`, pronto. Implica `--no-build` |
| `--no-build` | não reconstrói a imagem |
| `--no-odom-print` | cala a linha de deriva do VO a 1 Hz (o CSV continua) |

Qualquer argumento com `:=` vira **argumento de launch** e é repassado:

```bash
./scripts/docker_up.sh --phase1 --ground-truth target_bases:=2 takeoff_alt:=3.0
```

### Exemplos comuns

```bash
# a missão padrão, com as janelas de debug abertas
./scripts/docker_up.sh --phase1 --debug --ground-truth

# a mesma missão, voando na odometria de verdade (sem --ground-truth)
./scripts/docker_up.sh --phase1

# comparar as duas divisões de trabalho na mesma arena
./scripts/docker_up.sh --zed-detect --ground-truth

# outra arena: BASES_SEED sobrepõe a seed do config.yaml, sem rebuild
BASES_SEED=3 ./scripts/docker_up.sh --phase1 --ground-truth
```

### O que NÃO passa pelo docker_up

```bash
scripts/seed_sweep.sh 1 2 3 4 5 6      # voa a missão em N arenas e pontua cada uma
MISSION=--zed-detect scripts/seed_sweep.sh 1 2 3    # a outra missão
scripts/score_run.py --seed 3 --log <log>           # pontua UM log contra a verdade
```

E, no drone real, `scripts/jetson_up.sh` no lugar do `docker_up.sh`.

---

## 2. As duas camadas, e por que a separação existe

```
    SOURCES                          AUTONOMIA
    de onde vêm os dados             o que se faz com eles
    ─────────────────────────        ────────────────────────────
    sources_sim.launch.py     ->     phase1.launch.py
      (ou sources_real)                 (ou phase1_zed_detect)
```

A autonomia consome **apenas** os barramentos agnósticos — `/zed/zed_node/*`,
`/down_cam/*`, `/mavros/*` — e não sabe em que mundo está. Tudo que é simulado
é um **source**: ele produz exatamente os tópicos que o hardware produziria.

É por isso que `phase1_sim.launch.py` é um wrapper puro que **não passa
override nenhum**. Se você sentir vontade de acrescentar um, a correção
pertence ao lado dos sources — faça o simulador produzir o que o drone produz.

**A armadilha, medida em 2026-08-22:** um `DeclareLaunchArgument` repetido no
wrapper carrega o default *dele* e sobrescreve o do `phase1.launch.py`. Editar
o default no arquivo certo passava a não fazer nada. Por isso os wrappers
declaram só os argumentos que eles próprios possuem.

---

## 3. Os nós, por camada

### 3.1 Sources — o simulador (`sources_sim.launch.py`)

| nó | pacote | o que faz |
|---|---|---|
| `ardubridge_node` | biguasim_main | BiguaSim ↔ ArduPilot SITL (PWM entra, estado sai, FDM JSON) |
| `sitl_dds` + `micro_ros_agent` | ardupilot_sitl | o SITL em si. **Não suba um segundo agente** |
| `mavros_node` | mavros | MAVLink do SITL → `/mavros/*` |
| `zed_mimic_node` | hydrone_bringup | `/biguasim/*` → `/zed/zed_node/*`. Faz o papel do `zed_wrapper` |
| `down_cam_mimic_node` | hydrone_bringup | idem para a webcam de baixo → `/down_cam/*` |
| `rangefinder_bridge` | hydrone_bringup | LaserScan do sim → `DISTANCE_SENSOR` do MAVLink |
| `visual_odometry_node` | hydrone_localization | odometria visual de verdade sobre as imagens da ZED |
| `vision_odom_bridge` | hydrone_localization | `/zed/.../odom` → `/mavros/vision_pose/pose`. **Também roda no drone real** |
| `odom_error_node` | hydrone_bringup | compara VO contra verdade e grava CSV em `logs/` |

No drone real, `sources_real.launch.py` troca os `*_mimic` pelos drivers
(`zed_wrapper`, a câmera USB) e o SITL pelo Pixhawk. O resto é o mesmo.

### 3.2 Autonomia (`phase1.launch.py`)

| nó | pacote | o que faz | na missão padrão |
|---|---|---|---|
| `pad_detector_node` (forward) | hydrone_vision | acha pads na ZED, projeta com profundidade | **desligado** |
| `pad_detector_node` (down) | hydrone_vision | acha pads na webcam, projeta **lançando o pixel no octomap** | é o único detector |
| `pad_map_node` | hydrone_map | funde detecções → `/hydrone/pads/map` + marcadores | |
| `belly_coverage_node` | hydrone_map | pinta o que a barriga varreu + trajetória voada | observador puro |
| `feature_map_node` | hydrone_map | acumula a nuvem da ZED | observador puro |
| `cloud_filter_node` | hydrone_map | limpa a nuvem **antes** do octomap | |
| `octomap_server_node` | octomap_server | o mapa 3-D de ocupação | |
| `map_odom_node` | hydrone_localization | publica `map → odom` medido, unindo as duas árvores de TF | |
| `phase1_mission_node` | hydrone_mission | a máquina de estados: decola, varre, confirma, pousa, volta | |

**Ordem que importa:** `cloud_filter` fica entre a câmera e o `octomap_server` de
propósito. Apontar o octomap direto para a nuvem é a fiação óbvia e a errada —
um pixel voador a 18 m escava espaço livre através da parede a 4,86 m à qual
ele pertence.

### 3.3 O que a missão padrão faz, em duas passadas

```
1  PERÍMETRO fechado, quatro lados, volta ao ponto de partida
   produto: o octomap. Não detecções.

2  FAIXAS espaçadas pela pegada da webcam, calculada em tempo de execução
   do CameraInfo vivo e da altura sobre a superfície mais alta
   produto: as bases.
```

A posição de cada base vem de **lançar o pixel na octree** e pegar o primeiro
voxel ocupado — o topo da base, se for isso que está embaixo. Entrega 5–6 cm,
contra 1,06 m de um plano de chão assumido.

---

## 4. O que o `--debug` abre

```bash
./scripts/docker_up.sh --phase1 --debug --ground-truth
```

Duas janelas, **dentro do container**, desenhando no X desta máquina pelo mount
`/tmp/.X11-unix` que o compose já faz e pelo `xhost +local:docker` que o
`docker_up.sh` já roda. Não há `ROS_DOMAIN_ID` para casar nem nada a iniciar à
parte.

### rviz2, com `src/hydrone_bringup/rviz/phase1.rviz`

Fixed Frame `map` — **não `odom`**. Os dois diferem aqui por yaw +90° e
z −0,78 m, e escolher errado põe todo overlay no lugar errado sem erro nenhum.

| display | tópico | |
|---|---|---|
| Octomap | `/octomap/octomap_binary` | latched, ~3 KB. **Não** use os MarkerArray: são republicados inteiros a cada inserção e é isso que faz os cubos piscarem |
| Belly coverage | `/hydrone/belly/coverage` | o chão pintado. Um buraco aqui é faixa que ninguém olhou |
| Belly footprint | `/hydrone/belly/footprint` | o quadrilátero de visão atual |
| Flown trajectory | `/hydrone/belly/trajectory` | a linha voada |
| Pad map | `/hydrone/pads/markers` | as bases que a missão acredita ter achado |
| Planned route | `/hydrone/nav/plan` | para onde ela pretende ir |
| Vehicle pose | `/mavros/local_position/pose` | |
| TF, ZED cloud | | **desligados** — pesados e não usados por esta missão |

### rqt_image_view, em `/hydrone/pads/down/debug_image`

A visão **anotada** do detector, não o quadro cru. É a janela que responde "o
detector está vendo o que eu acho que ele está vendo" — que a §13 do
`PHASE1-MISSION.md` manda olhar antes de mexer na máquina de estados.

Para a missão `--zed-detect`, aponte para a frontal:

```bash
./scripts/docker_up.sh --zed-detect --debug debug_image_topic:=/hydrone/pads/forward/debug_image
```

---

## 5. Ferramentas que não são nós

| script | para que |
|---|---|
| `seed_sweep.sh` + `score_run.py` | voa N arenas e pontua cada uma contra a verdade da seed |
| `view_topic.py` | ver um tópico de imagem sem rqt. É o que roda **do host** contra o drone real |
| `rviz_remote.sh` / `view_remote.sh` | rviz e visualizador **na sua máquina**, olhando o drone real pela rede |
| `depth_probe.py` | profundidade por pixel, em metros |
| `hsv_probe.py` | HSV de uma região — para retunar o detector |
| `odom_report.py` | lê o CSV de deriva do VO |
| `dev_rebuild.sh` | rebuild em segundos dentro do container, para `.msg`, `setup.py` e arquivos novos |

---

## 6. Coisas que custaram tempo e não devem ser redescobertas

- **`docker_up.sh` reconstrói por padrão.** Edição não commitada **voa**. Passe
  `--no-build` se quiser exatamente o que está commitado.
- **Não rode `docker compose exec` contra o container durante uma corrida.**
  Aqui isso derrubou o `docker compose up` com `exit 137`, que parece crash do
  simulador e não é. Use `docker exec joao_pessoa_2026-hydrone-1 ...`.
- **Cada bringup escreve um `.utrace`** em `~/UnrealEngine/UnrealTrace/Store/`.
  Uma corrida longa deixou **87 GB sozinha**, e um mês delas acumulou 512 GB e
  levou o disco a 100% — ponto em que nada na máquina consegue escrever. O
  `docker_up.sh` agora os limpa antes de cada subida.
- **`source scripts/env.sh` antes de qualquer ferramenta ROS no host** — ele
  casa o `ROS_DOMAIN_ID` com o do compose (63). Com o valor errado o rviz não
  lista nada enquanto o stack publica centenas de tópicos.
- **Arquivo de launch ou config NOVO exige build**, mesmo em `--dev`: os links
  simbólicos são criados por arquivo no momento do build.
- **29 testes de `test_phase1_mission.py` falham e são pré-existentes** — o
  arquivo ficou para trás quando o estado `ROTATE` e os níveis 3 e 4 saíram.

## Relacionados

- [`MAP-SWEEP-2026-09-02.md`](MAP-SWEEP-2026-09-02.md) — a missão padrão, e por que ela é assim
- [`SEED-SWEEP-2026-09-02.md`](SEED-SWEEP-2026-09-02.md) — ela medida em sete arenas
- [`NAV-E-DETECCAO-2026-09-02.md`](NAV-E-DETECCAO-2026-09-02.md) — a detecção da ZED
- [`PHASE1-MISSION.md`](PHASE1-MISSION.md) — a máquina de estados em detalhe
- [`PACKAGES.md`](PACKAGES.md) — por que os pacotes estão divididos assim
