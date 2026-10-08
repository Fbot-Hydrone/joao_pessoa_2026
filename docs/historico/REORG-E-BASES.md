# Bases sorteadas e a reorganização dos pacotes

Duas mudanças que entraram juntas na branch `spawn-bases-seed`, feitas em
2026-08-26. Não têm relação entre si — a primeira é uma funcionalidade nova, a
segunda é movimentação de código — mas quem for ler o diff vai ver as duas.

---

## 1. Bases de pouso sorteadas por seed

A Fase 1 põe 6 bases móveis em qualquer lugar da arena, alocadas
aleatoriamente antes da fase começar. Para testar contra isso, a simulação
precisava spawnar bases em posições que mudam — mas de forma repetível, senão
não se compara uma corrida com a outra.

### Onde isso vive

`src/biguasim-ros2/biguasim_main/biguasim_main/bases.py` — o sorteio, em
Python puro, sem ROS e sem BiguaSim. A geometria da arena vem de fora, por
parâmetro, então a função é testável isoladamente.

O spawn em si acontece em `interface.py::spawn_bases()`, e é o
`ardubridge_node` que a chama. **Isso importa:** o `phase1_sim` não sobe o
`biguasim_node`, sobe o `ardubridge_node`, e ele cria a interface com
`init=False` porque monta o env por conta própria via `ArduBiguaSimRunner`. Um
gancho no ramo `init=True` da interface nunca executa nesse caminho — foi o
primeiro bug desta implementação, e o sintoma foi o sim subir limpo, sem erro
e sem base nenhuma.

### O que se configura

Em `biguasim_main/config/config.yaml`, dentro de `biguasim_scenario`:

```yaml
bases:
  seed: 42
  count: 6
  blueprint: "Blueprint'/Game/Maps/arena__2_/BP_base.BP_base_C'"
  z_min: 0.0
  z_max: 1.5          # regra: base móvel entre 0 e 1,5 m do solo
  min_spacing: 1.5    # distância mínima entre centros, em metros
  house: [-4.0, 2.0, 2.0, 4.0]
  house_height: 1.5
  takeoff: [2.0, 4.0, 2.0, 4.0]
```

Trocar `seed` troca a arena inteira, de forma determinística. Apagar o bloco
`bases` inteiro desliga o spawn — sem o bloco, `spawn_bases()` é um no-op.

O bloco é retirado do scenario (`scenario.pop('bases')`) antes do
`biguasim.make()`, porque `bases` não faz parte do schema do BiguaSim.

### As regras que o sorteio respeita

Arena de 8×8 m com centro em (0,0,0), então `x,y ∈ [-4, 4]`.

| regra | como é aplicada |
|---|---|
| a ≥ 0,5 m das paredes | sorteio limitado a `[-3.5, 3.5]` |
| altura de 0 a 1,5 m | `z` sorteado em `[z_min, z_max]` |
| nada na base de decolagem | região `takeoff` rejeitada, inflada em 0,5 m para a base de 1×1 m caber inteira fora |
| pode ficar em cima da casinha | um ponto sobre a `house` só vale se a base couber **inteira** no telhado; aí `z` vira `house_height` em vez do valor sorteado |
| bases não se sobrepõem | `min_spacing` entre centros |

Encostou na casinha mas não cabe inteira no telhado — borda ou parede — o
ponto é rejeitado. Nunca existe base *dentro* da casa.

### O Y invertido

**O `SpawnMesh` do mapa usa o eixo Y com sinal oposto ao do `location` do
agente.** O `interface.py` envia `[x, -y, z]` por causa disso.

Sem essa inversão o sorteio parece certo e o resultado é errado: o filtro de
região trabalha no frame medido (o mesmo do `location` do drone), a base vai
parar no Y espelhado, e uma base sorteada em `y = -2.89` — livre — aparece em
`+2.89`, em cima da base de decolagem. Foi o segundo bug, e o que o denunciou
foi espelhar as coordenadas do log contra o viewport: as três bases que caíam
em região proibida depois do espelhamento eram exatamente as três que estavam
erradas na tela.

### Como verificar

O `ardubridge_node` loga as posições na subida:

```
6 bases spawnadas (seed 42): [0.98, -3.32, 0.41], [2.75, -2.89, 0.63], ...
```

`seed: 42` não exercita o telhado — nenhuma das 6 cai sobre a casinha. Para
esse caso use `seed: 0`, que põe duas em cima, ambas a exatamente 1,5 m.

---

## 2. Reorganização dos pacotes

O `hydrone_bringup` era o maior pacote de código do stack (2754 linhas) *e* o
dono dos 12 launches; o `hydrone_nav`, que existe para planejar rota, era 75%
mapa. Ver [`PACKAGES.md`](PACKAGES.md) para a estrutura final e as regras.

### O que se moveu

| de | para | o quê |
|---|---|---|
| `hydrone_nav` | **`hydrone_map`** (novo) | `pad_map_node`, `feature_map_node` |
| `hydrone_bringup` | **`hydrone_localization`** (novo) | `visual_odometry_node`, `map_odom_node`, `vision_odom_bridge` |
| `hydrone_mission` | `hydrone_nav` | a escolha de alvo, como `route.py` |

Tudo com `git mv`, então `git log --follow` continua encontrando o histórico.

O `hydrone_bringup` fica com o que ele é: de onde vêm os dados (os pares
sim/real `zed_mimic`↔`zed_sdk`, `down_cam_mimic`↔`down_cam_usb`), o
`rangefinder_bridge`, o `odom_error` e **todos os launches**.

### `hydrone_nav.route`

`_best_candidate`, `_is_candidate` e `_takeoff_base_xy` eram regras de escolha
puras presas num nó de 1441 linhas. Viraram uma biblioteca que **não importa
ROS** — nem `rclpy`, nem `hydrone_msgs`: trabalha por duck-typing nos campos do
`Pad`. Uma missão da Fase 2 reusa a escolha de alvo sem copiar um mission node
inteiro, e o teste dela roda com `SimpleNamespace`.

Os métodos do nó continuam existindo como fachada fina, delegando à
biblioteca. Por isso os testes de missão seguiram valendo sem edição.

### A lista de pacotes

Estava escrita à mão em três arquivos, e criar dois pacotes exigiu editar os
três. `Dockerfile` e `scripts/dev_rebuild.sh` passam a descobrir os pacotes de
`src/hydrone_*` e `src/biguasim-ros2` — `src/ardupilot` fica de fora por não
ser nomeado. Só `docker-compose.dev.yml` continua explícito, porque YAML não
tem glob.

### O que NÃO se moveu

`vision_node`, `mission_node`, `pad_mission_node`, `nav_node` e
`controller_node` (~2500 linhas) são a geração anterior e não estão no caminho
do `phase1_sim`. Continuam buildando, com seus launches. Apagá-los é decidir o
que ainda se quer poder rodar, não refatorar.

---

## Verificação

- **235 testes verdes** (224 antes da reorganização, mais 11 unitários novos da `route`)
- **Build completo da imagem**: os 10 pacotes instalam
- **Os 12 launches resolvem** (`--show-args`), e todo par `package=`/`executable=`
  existe no pacote declarado
- **Voo em `phase1_sim --ground-truth`**: os 8 nós sobem, sem erro de import, e
  o drone visita as bases

### O que ainda não foi provado

Uma rodada **sem** `--ground-truth`. Com ground truth, o `odom_wiring` manda o
`visual_odometry_node` para `/zed/zed_node/odom_VO` com `publish_tf=false`:
ele sobe e processa, mas quem alimenta o EKF é o `zed_mimic` com a verdade do
simulador. O VO real fechando a malha — o caminho do drone de verdade — é o
único trecho ainda não exercitado depois da mudança de pacote.

```
BS_SIM_DIR=... ./scripts/docker_up.sh --phase1
```

## As ferramentas de debug

Nenhuma mudou de nome, de tópico ou de invocação:

| ferramenta | estado |
|---|---|
| `--ground-truth` | intacto — `odom_wiring` não foi tocado |
| `--no-odom-print` / CSV do `odom_error` | intacto — o nó ficou no `hydrone_bringup` |
| `phase1_dry.launch.py` | resolve |
| `landing_sites*.launch.py`, `hydrone_sim`, `hydrone_mission_sim` | resolvem |
| `scripts/rviz_remote.sh`, `view_remote.sh`, `view_topic.py` | não citam pacote nenhum — trabalham por tópico |
| `/hydrone/pads/*/debug_image` | intacto — `pad_detector` não foi tocado |
| `scripts/dev_shell.sh`, `dev_rebuild.sh` | `dev_rebuild` agora descobre os pacotes sozinho |
