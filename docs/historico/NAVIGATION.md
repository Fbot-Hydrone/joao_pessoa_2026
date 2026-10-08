# A navegação do drone — o que existe, por que existe, e o que ainda não funciona

Escrito em 2026-08-29, no fim de uma sessão longa. Substitui e amplia
`PHASE1-SEARCH.md`.

**Leia isto antes de mexer em qualquer coisa da busca.** Quase toda decisão
aqui tem um número atrás dela, e vários dos comportamentos que parecem
arbitrários são o resultado de uma coisa mais simples ter sido tentada e
medida como pior.

---

## 0. O aviso que vale mais que o resto

**Tudo abaixo foi desenvolvido e medido com `--ground-truth`.**

| | erro mediano de posição |
|---|---|
| voando no ground truth | 1,21 m |
| voando no VO | **7,83 m** |

Numa arena de 8 m. O drone real não tem ground truth. A navegação está
completa e funciona; ela está construída sobre uma estimativa de pose que,
no caminho real, ainda erra quase a arena inteira. Ver `NAV-2026-08-27.md`.

---

## 1. A estratégia, em cinco passos

Está toda em `_do_settle` do `phase1_mission_node`. A ordem **não é gosto, é
histórico de falha** — cada passo existe porque o anterior foi medido
insuficiente.

```
1. voa o levantamento (a escada de níveis)
2. pousa no que achou, melhor candidata primeiro
3. abaixo da cota? investiga o RELEVO do octomap
4. ainda abaixo? olha de outro lugar
5. senão, volta para a base de decolagem — nunca pousa onde está
```

### O princípio: levantar primeiro, comprometer depois

Correr atrás da primeira detecção é explorar-nada/explorar-tudo na pior
ordem: a bateria vai para a base que por acaso estava na frente da câmera na
decolagem, e as que ele nunca virou para olhar não são achadas nunca.

**Essa barreira precisa existir em dois lugares.** O `TAKEOFF` entrega direto
no `SELECT`, então o `SELECT` é uma *segunda porta* para a fase de pouso.
Ficou destrancada por muito tempo sem que ninguém notasse, porque o mapa está
vazio na decolagem e a missão caía na busca **por acidente**. No dia em que
as detecções passaram a ser aceitas com o veículo em movimento, ela voou para
**quatro bases antes da varredura começar**.

## 2. A escada de busca

Cada nível custa mais que o anterior. Só se gasta o próximo se a cota não
fechou.

| nível | forma | giros | por que existe |
|---|---|---|---|
| **1** | U a `survey_alt_m` (2,0 m), três lados | 2, só nos cantos | forma mais barata que vê o chão inteiro |
| **2** | o **mesmo** U a +`level2_climb_m` | 2 | base tapada pela casinha abre de mais alto |
| **3** | girar no lugar + investigar relevo | vários | onde a base **elevada** é pega |
| **4** | corta-grama, faixas de `lawnmower_lane_m` | muitos | último recurso |

Entre um nível e o próximo, **as pistas não confirmadas são investigadas
primeiro** — uma hover resolve uma pista, um nível inteiro custa minutos.
Durante a investigação a barra cai para **uma** observação: não porque a barra
esteja errada no resto do tempo, mas porque o custo de errar mudou. Durante a
busca uma pista duvidosa concorre com achar base de verdade; depois que a
busca empacou, ela concorre apenas com voar o U de novo.

### Por que voar em vez de girar

O que limita o mapa **não é para onde a câmera aponta, é onde ela tem
paralaxe**. Uma câmera que nunca translada nunca vê atrás de nada.

Isso foi medido do jeito difícil. Uma versão "dirigida" da varredura
perguntava quais direções tinham arena não observada atrás delas — e uma
arena aberta responde *"todas"*. Ela voltou `22, -22, 68, -68, 112, -112,
158, -158`: um círculo completo com os giros apenas reordenados.

### Por que três lados e não quatro

Do terceiro lado a câmera já olha de volta por cima do que o quarto cobriria.
O quarto seria bateria gasta refotografando o mapa.

### Por que parar antes de girar

Um setpoint que muda posição **e** rumo ao mesmo tempo pede ao veículo que
gire ainda em movimento, e girar sob translação é onde a odometria desta
arena mais perde. Cada canto é dois setpoints na **mesma posição**: a perna
que chega ali, e depois um giro puro, parado.

### Por que a perna inteira é um setpoint só

No GUIDED do ArduCopter um alvo de posição significa *"vá até lá e **pare**"*.
A missão só solta o próximo depois de chegar, então cada ponto intermediário
era um ciclo completo de frear e acelerar.

Com `WP_SPD 1.5` e `WP_ACC 1.5`, chegar à velocidade de cruzeiro leva 1 s e
0,75 m, e parar o mesmo. Com o espaçamento de 1,5 m que havia, **o veículo
nunca alcançava a velocidade de cruzeiro em lugar nenhum da varredura** —
média de 0,75 m/s contra uma fuselagem capaz do dobro.

E os pontos intermediários não tinham função: a perna do U é uma reta voada
num rumo só. Eram herança do retângulo anterior, onde o yaw *era* re-mirado a
cada passo.

### Por que a pausa encolheu

O `settle_s` existe por **um** motivo, escrito na descrição dele: uma detecção
tirada com o yaw ainda girando é projetada por uma estimativa em movimento e
cai metros fora. É guarda contra **girar**.

A varredura não gira. Pagar a pausa inteira em cada waypoint era pagar por um
perigo que não estava lá: 23 waypoints × 5 s = 115 s dos ~200 s do nível.
Agora é pausa cheia depois de mudança de rumo, `settle_moving_s` depois de
translação pura.

**Resultado acumulado das três mudanças acima:**

```
varredura do nível 1     ~200 s  ->  92 s  ->  70 s
```

## 3. De onde vêm as candidatas

Duas fontes, respondendo perguntas **diferentes**:

```
detector azul        "isto parece um pad"     -> pad_map, por detecção
relevo do octomap    "tem algo em pé aqui"    -> pista a investigar
```

O octomap **não sabe** o que é uma base — ele guarda ocupação, e um pad azul e
o chão branco embaixo são o mesmo voxel ocupado. O que ele sabe, e nada mais
no stack sabe, é **onde ninguém olhou** e **onde há relevo**.

O relevo importa por um motivo específico: a projeção no plano do chão cruza o
raio com `z = chão`, então **uma base a 1,5 m é posicionada errada por melhor
que seja vista**. Girar não ajuda — o modelo de projeção é que está errado
para ela. O octomap mede onde a matéria está, em 3-D, e não tem esse problema.

## 4. As barreiras que nunca podem cair

Pousar fora de base é **eliminatório**.

1. **Uma perna de OLHAR nunca confirma nem pousa.** Chegar a um ponto de
   observação é o fim da viagem, não o começo de um pouso.
2. **Nada desce sem `target_id`.** `over pad None` é como o veículo termina no
   chão.
3. **Detecção fora da arena é recusada** no instante em que a posição é
   calculada.
4. **Perna bloqueada sem caminho é recusada**, não voada "confiando no
   supervisor".
5. **Cerca no `_stream`** — o único ponto por onde um setpoint chega ao FCU.

Cada uma dessas veio de uma falha real:

- MEDIDO 2026-08-27: uma corrida que reportou "6 de 6 bases" tinha pousado em
  **uma**. As outras cinco foram no meio da arena, sobre o que a barriga topou
  ver — a perna de cobertura chegava e era tratada como chegada sobre um pad.
- MEDIDO 2026-08-28: candidatas aceitas em `(10.23, -2.25)` numa arena que vai
  até 4 m. Um raio mirado no alto de uma parede cruza o plano do chão **do
  lado de lá dela**.
- O "flying the straight line and relying on the supervisor" produziu uma
  batida na parede. Não existe entrada de supervisor dentro de uma perna de
  2 m em cruzeiro.

## 5. A centragem na base

Durante a hover de confirmação o veículo se centraliza sobre o pad usando a
câmera de baixo. **O mapeamento de pixel para metro não é constante** — é uma
rotação, um sinal e uma escala, e nenhum dos três pode ser anotado:

- a escala muda com a altura, a cada frame de uma descida;
- a rotação depende de como a câmera foi parafusada, e a do drone real não é a
  do simulador;
- o sinal segue da rotação, e errá-lo **não centraliza devagar — afasta o
  veículo, acelerando**.

MEDIDO no simulador, 42 passos: `dv` acompanhou `body_x` a **+0,810** e `du`
não acompanhou nada (0,00002 e 0,00014). Mesmo com a montagem escrita num
arquivo de configuração não há constante limpa.

Então `hydrone_nav/servo.py` **sonda**: duas sondagens ortogonais determinam a
jacobiana 2×2 **exatamente**, para qualquer montagem inversível. Broyden refina
depois. (Broyden sozinho, de um palpite diagonal, **divergia para 1,66 m** numa
montagem girada 90° — é para isso que as sondagens existem.)

**O que ele não consegue aprender:** para onde a câmera aponta com o veículo
nivelado — isso é o que "centralizado" *significa*. Numa fuselagem
desalinhada, centralizar no meio da imagem estaciona o drone fora da base pelo
desalinhamento × altura. É o parâmetro `pad_target_uv`, medido uma vez pairando
sobre uma base conhecida. **É o único número a calibrar no drone real.**

## 6. Os parâmetros, e os que estão acoplados

Tudo em `phase1.launch.py`. Editar o `default_value` funciona; com `--dev` a
edição vale na hora (o launch é symlink até `/ws/src`, que está bind-montado),
sem `--dev` precisa reconstruir a imagem.

**A arena é uma fonte só.** `arena_size_x`, `arena_size_y`, `arena_centre_x`,
`arena_centre_y` alimentam ao mesmo tempo a caixa do planejador, o comprimento
das pernas do U, as faixas do corta-grama e a parede do `pad_map`. Uma arena
de 5×6 é `arena_size_x:=5.0 arena_size_y:=6.0` e nada mais.

`arena_centre` existe porque o `map` do drone real é **onde ele armou** — uma
base de decolagem em algum lugar da arena, não o centro dela.

### Dois acoplamentos que mordem em silêncio

- **`WP_SPD` × `max_map_speed`.** A velocidade de cruzeiro está em
  `config/params/holybro_sitl.parm` (`WP_SPD 1.5`), **não** no launch. O
  `max_map_speed` do `pad_map` precisa ficar **acima** dela, senão a varredura
  voa o tempo todo acima do gate e o mapa não aprende nada — MEDIDO: 39
  detecções recusadas contra 4 aceitas, e a missão voltou com 4 de 6 bases.
- **`survey_alt_m` × altura da casinha.** As passadas cruzam a casinha, cujo
  telhado é 1,5 m. Uma varredura no `takeoff_alt` de 1 m voaria para dentro
  dela.

## 7. O que NÃO está resolvido

Em ordem do que mais ameaça a corrida.

### 7.1 Decolagem recusada em base elevada — **o bloqueio principal**

Depois de pousar numa base a ~0,9-1,1 m, o FCU recusa `NAV_TAKEOFF` e só diz
`no reason given`. A missão aborta corretamente (o livelock que fazia ela
tentar para sempre está consertado), mas termina ali.

Com 6 bases e várias elevadas isso mata a corrida **no primeiro pouso alto**,
antes que qualquer outra coisa importe. Não diagnosticado.

### 7.2 Base elevada não é achada

Trava a seed 7 em 5 de 6. A projeção no plano do chão é estruturalmente errada
para elas; o detector prefere profundidade quando existe, mas a
`depth_registered` roda a **2,5 Hz** contra 3,3 Hz da RGB, então muitos frames
caem no fallback.

Duas saídas: sincronizar RGB e profundidade, ou promover o relevo a fonte de
candidata desde o nível 1 em vez de esperar o nível 3.

**E cuidado:** o nível 2 (subir 0,5 m) é a coisa errada para esse modo de
falha — subir **aumenta** o erro da projeção. O nível 2 está certo para "base
tapada pela casinha" e errado para "base alta".

### 7.3 O VIO

Ver a seção 0. É o teto de tudo.

### 7.4 Sem relógio de tentativa

A Fase 1 dá 3 tentativas em 30 minutos e a missão **não tem noção de tempo**.
Ela pode gastar a tentativa inteira no nível 4 e pousar em nada — zero ponto
com o mapa cheio de bases confirmadas. É barato de fazer e é a diferença entre
pontuar e não pontuar num voo ruim.

### 7.5 Ordem de pouso

`route.nearest_candidate` escolhe o mais próximo a cada vez, que não é o
percurso mais curto. Com 6 bases e prazo apertado, a ordem vale ~1 pouso.

## 8. Como verificar

```bash
BS_SIM_DIR=... ./scripts/docker_up.sh --phase1 --ground-truth target_bases:=6
```

No log, o que olhar:

```
SEARCH LEVEL 1: ...            a varredura começou
SEARCH LEVEL 1 found all N     fechou sem escalar
LANDED on base #N of M         pouso real, sempre precedido de CONFIRMED
over pad None                  NUNCA deve aparecer — é pouso fora de base
outside the arena              detecção atrás da parede, recusada
vehicle is translating         se aparecer muito, max_map_speed < WP_SPD
setpoint ... outside the arena  a cerca mordeu
```

Testes: 485, e os pacotes ROS-free rodam no host em centésimos de segundo:

```bash
PYTHONPATH=src/hydrone_nav /usr/bin/python3 -m pytest src/hydrone_nav/test -q
```

## Relacionados

- [`NAV-2026-08-27.md`](NAV-2026-08-27.md) — o VIO, o giroscópio 3x, e a
  medição negativa do landmark
- [`OCTOMAP.md`](OCTOMAP.md) — o mapa de ocupação
- [`PHASE1-MISSION.md`](PHASE1-MISSION.md) — a máquina de estados da missão
