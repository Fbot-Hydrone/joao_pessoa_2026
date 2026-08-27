# Onde o stack está — ponto de partida para a próxima sessão

Escrito em 2026-08-27, no fim de uma sessão longa. Branch `spawn-bases-seed`,
tudo commitado e no remoto. 307 testes verdes.

---

## Localização ≠ navegação (a pergunta do octomap)

**"Pra que criar um octomap se não vamos usar?"** — ele vai ser usado, só que
para outra coisa.

São dois problemas diferentes e eles querem estruturas de dados diferentes:

| pergunta | quem responde | por quê |
|---|---|---|
| **Onde eu estou?** | landmarks (o `pad_map`) | as bases são únicas e esparsas |
| **Por onde eu vou?** | o octomap | ele sabe o que é livre, ocupado e desconhecido |

O octomap é ruim para *localização* **nesta arena específica**: quatro paredes
brancas idênticas, planta quadrada com simetria de 90°, nenhuma textura. Um
ICP contra isso desliza ao longo da parede e confunde um canto com o outro.

Mas ele é a única coisa no stack que responde **"esse caminho está livre?"** —
e nenhum mapa de landmarks responde isso. A Fase 4 é navegação autônoma num
espaço confinado e escuro, atrás de 5 alvos: é literalmente planejamento de
trajetória em 3-D, e é para isso que o octomap existe.

Então: o octomap fica, e cresce para o planejador. A correção de pose vem dos
landmarks. Não competem.

---

## O que existe hoje

### Pacotes (ver [`PACKAGES.md`](PACKAGES.md))

Sete, divididos por *o que a coisa é*, não por fase:

```
hydrone_bringup        de onde vêm os dados + todos os launches
hydrone_localization   onde o drone acha que está  (VIO)
hydrone_map            o que ele lembra do mundo   (pads, nuvem, octomap)
hydrone_nav            como chegar lá              (route.py)
hydrone_vision         o que a câmera vê
hydrone_mission        o que fazer, em que ordem
hydrone_controller     órfão do caminho da Fase 1
```

Regra que sustenta isso: **biblioteca não importa ROS**. `hydrone_nav.route`,
`hydrone_map.cloud_filter`, `hydrone_map.octree` e `hydrone_vision.pad_detector`
são Python puro, testáveis com fakes e reusáveis por qualquer fase.

### Bases sorteadas ([`REORG-E-BASES.md`](REORG-E-BASES.md))

`seed` no `config.yaml` → 6 bases, sempre nos mesmos lugares para a mesma seed.
Respeita as regras: 0,5 m das paredes, 0 a 1,5 m de altura, nada na base de
decolagem, e em cima da casinha só se couber inteira no telhado (a 1,5 m).

**Armadilha registrada:** o `SpawnMesh` usa Y com sinal oposto ao `location` do
agente. O `interface.py` envia `[x, -y, z]`.

### Octomap ([`OCTOMAP.md`](OCTOMAP.md))

Ligado por padrão, tudo sob `/octomap/`. Voxel de 0,15 m, 2 Hz, latched.
Consultável de Python via `hydrone_map.octree`:

```python
tree = tree_from_msg(msg)              # de /octomap/octomap_binary
query(tree, (x, y, z))                 # occupied | free | UNKNOWN
path_is_clear(tree, a, b)
```

**Para olhar no rviz**, o plugin tem que estar **no host**:
`sudo apt install ros-humble-octomap-rviz-plugins`, display
`octomap_rviz_plugins/OccupancyGrid` em `/octomap/octomap_binary`. **Não** use
os MarkerArray — cada publicação manda 16 `DELETE` e 1 `ADD`, e é isso que faz
os cubos piscarem.

### Odometria ([`VO-DRIFT.md`](VO-DRIFT.md))

Quatro mudanças, nesta ordem:

1. **ZUPT.** 94% das amostras o drone estava parado e o VO reportava 1,61 mm
   por frame — 442 m de distância inventada num voo de 51 m. Nenhum gate
   existente pegava: eles rejeitam frames RUINS, e uma câmera parada não falha
   em nenhum. Resultado: inflação de 10,2x → 1,02x, erro de yaw de 180° → 4,2°.
2. **CLAHE + `fastThreshold` 20→7.** A arena não é sem textura, é de baixo
   contraste. Medido num frame real: 59 → 917 keypoints, espalhados por 225
   linhas em vez de 51.
3. **Par estéreo.** Duas RGB (baseline 0,12 m), profundidade do match entre
   elas, busca na linha epipolar. **Só para o odom** — mapa, detectores e
   `odom_GT` seguem com a depth do sim.
4. **VIO.** O giroscópio entra em três pontos: semeia o PnP, veta soluções
   visuais que contradiz, e carrega a pose quando a visão falha. Só rotação;
   translação não é dead-reckoned de propósito.

---

## O que está verificado e o que NÃO está

**Verificado, com o sim voando:**

- as bases spawnam nas coordenadas certas
- o octomap constrói a arena (paredes, chão, casinha visíveis no rviz)
- o ZUPT e o CLAHE melhoraram a odometria de forma grande e medida

**NÃO verificado:**

- **O VIO nunca voou.** Foi implementado e testado unitariamente (integração do
  giro, conversão de frame, ausência de dead reckoning), mas o sim caiu antes
  de qualquer voo com ele. **Esta é a primeira coisa a fazer.**
- **O estéreo, medido, ficou PIOR que a depth do sim** (7,68 m contra 1,58 m).
  A causa é geometria, não código: `fx=320` e `B=0,12` dão 0,23 m de erro por
  pixel de disparidade a 3 m e 0,94 m a 6 m — a faixa onde o drone voa. A ZED
  real tem `fx≈700`. Ver [`VO-DRIFT.md`](VO-DRIFT.md) para as três saídas.
  **O VIO pode mudar esse número**, porque o giro veta justamente as soluções
  ruins que a triangulação imprecisa produzia. Medir antes de decidir.
- **`in_imu` pode estar no tópico errado.** O default é
  `/zed/zed_node/imu/data`; a fonte crua é
  `/biguasim/uav0_id0/DynamicsSensor/IMU`. Não deu para confirmar qual publica
  e a que taxa. É parâmetro, não precisa recompilar.

---

## Próximos passos, em ordem

### 1. Voar o VIO e ler o CSV

```
BS_SIM_DIR=... ./scripts/docker_up.sh --phase1
ls logs/odom_error_*.csv
```

Os CSVs agora caem em `./logs/` no host (antes morriam dentro do container).
No log, procurar `carried N deg on the gyro` e `PnP says X, gyro says Y` — é a
fusão agindo. As colunas que importam: `err_norm` e `drift_pct`.

Esse número decide os passos 2 e 3.

### 2. Localização por landmark (o "SLAM" que faz sentido aqui)

Não é scan matching. O `pad_map` já fusiona detecções com `confidence`,
`observations` e `is_takeoff_base`. A correção cai de graça:

```
o pad_map diz:   a base 3 está em (2.02, -3.24)
a detecção diz:  dessa pose, ela aparece em (2.41, -3.02)
a diferença:     0,45 m — esse É o erro da pose
```

Ordem: **âncora na base de decolagem** primeiro (a posição dela não veio do
mapa, veio de onde o drone armou — é a única âncora absoluta), depois correção
por re-observação de pads maduros.

**A armadilha:** o `pad_map` foi construído A PARTIR da pose. Corrigir a pose
com ele realimenta o erro. Defesa: só corrigir com landmarks de muitas
observações cuja posição parou de mudar, e dar peso diferente à base de
decolagem.

### 3. Navegação sobre o octomap

O que falta, de [`OCTOMAP.md`](OCTOMAP.md):

- **`filter_speckles` está off** — voxel isolado de ruído vira obstáculo
  fantasma. Uma linha.
- **Não existe inflação de obstáculo.** O mapa diz que um voxel está ocupado;
  não sabe que o drone tem 330 mm e não passa raspando. **Obrigatório** antes
  do primeiro voo autônomo em espaço confinado.
- **`path_is_clear` não é um planejador.** Falta o A*/RRT sobre a octree, e
  isso é trabalho do `hydrone_nav`.

### 4. Limpeza pendente

- ~2500 linhas da geração anterior (`vision_node`, `mission_node`,
  `pad_mission_node`, `nav_node`, `controller_node`) não estão no caminho do
  `phase1_sim`. Continuam buildando. Apagar é decidir o que ainda se quer poder
  rodar.
- `feature_map_node` e o octomap consomem a mesma nuvem fazendo trabalho
  sobreposto, e o `feature_map` é observador puro que nada lê. Desligá-lo por
  padrão é a maior economia fácil que sobrou.

---

## Coisas que custaram caro e não devem ser redescobertas

- **`ROS_LOCALHOST_ONLY` + `ROS_DOMAIN_ID` juntos** isolam o DDS do container
  do host: o rviz não lista nada enquanto o stack publica 176 tópicos. Está
  comentado no `docker-compose.yml` com o motivo.
- **O `env.sh` tem que casar o `ROS_DOMAIN_ID` do compose** (hoje 63). Estava em
  42 e isso sozinho fazia o rviz não ver nada. `source scripts/env.sh` antes do
  rviz.
- **`HYDRONE_LAUNCH_ARGS` precisa estar no `command:` do compose**, senão todo
  `name:=value` passado ao `docker_up.sh` é descartado em silêncio — enquanto o
  script ainda imprime "Launch args" como se tivesse funcionado.
- **O default de `odom_error_dir` tem que estar nos 5 launches de topo**, não só
  no `sources_sim`: cada um declara o argumento e repassa o seu.
- **`pointcloud_min_z` não existe** no octomap_server 2.3.1 (é
  `point_cloud_min_z`). O nó aceita o nome errado como override não declarado e
  **ignora** — o clamp nunca acontece e não há erro.
- **O `ardubridge_node` cria a interface com `init=False`**, então um gancho no
  ramo do `biguasim.make()` nunca executa no caminho da Fase 1.
