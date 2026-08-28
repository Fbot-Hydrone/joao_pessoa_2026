# Como o drone procura e pousa nas bases

Escrito em 2026-08-28, depois da primeira corrida que fechou 6 de 6.

A estratégia inteira está em `_do_settle` do `phase1_mission_node`, em cinco
passos. Cada passo existe porque o anterior foi **medido** insuficiente — a
ordem não é gosto, é histórico de falha.

---

## O princípio: levantar primeiro, comprometer depois

Correr atrás da primeira detecção é explorar-nada/explorar-tudo na pior ordem:
a bateria vai para a base que por acaso estava na frente da câmera na
decolagem, e as que ele nunca virou para olhar não são achadas nunca.

Então: varre a arena, monta o mapa de candidatas, **depois** pousa.

## A escada de busca

Cada nível custa mais que o anterior. Só se gasta o próximo se a cota não
fechou.

| nível | forma | giros | por que existe |
|---|---|---|---|
| **1** | U a `survey_alt_m` (2,0 m), três lados | 2, só nos cantos | forma mais barata que vê o chão inteiro |
| **2** | o **mesmo** U a +`level2_climb_m` (2,5 m) | 2 | base de perfil vista de baixo, ou tapada pela casinha, abre de mais alto |
| **3** | girar no lugar + investigar relevo | vários | onde a base **elevada** é pega |
| **4** | corta-grama, faixas de `lawnmower_lane_m` | muitos | último recurso |

Entre um nível e o próximo, **as pistas não confirmadas são investigadas
primeiro**. Uma hover resolve uma pista; um nível inteiro custa minutos.

### Por que três lados e não quatro

Do terceiro lado a câmera já olha de volta por cima do que o quarto cobriria.
O quarto seria bateria gasta refotografando o mapa.

### Por que voar em vez de girar

O que limita o mapa não é para onde a câmera **aponta**, é onde ela tem
**paralaxe**. Uma câmera que nunca translada nunca vê atrás de nada.

MEDIDO: perguntado quais direções tinham arena não observada atrás delas, uma
arena aberta responde "todas" — a varredura "dirigida" voltou
`22, -22, 68, -68, 112, -112, 158, -158`, um círculo completo com os giros
apenas reordenados.

### Por que parar antes de girar

Um setpoint que muda posição **e** rumo ao mesmo tempo pede ao veículo que gire
ainda em movimento, e girar sob translação é onde a odometria desta arena mais
perde. Cada canto emite a mesma posição duas vezes: chega no rumo antigo,
depois gira parado.

## De onde vêm as candidatas

Duas fontes, e elas respondem perguntas diferentes:

```
detector azul        "isto parece um pad"        -> pad_map, por detecção
relevo do octomap    "tem algo em pé aqui"       -> pista a investigar
```

O octomap **não sabe** o que é uma base — ele guarda ocupação, e um pad azul e
o chão branco embaixo são o mesmo voxel ocupado para ele. O que ele sabe, e
nada mais no stack sabe, é **onde ninguém olhou** e **onde há relevo**.

O relevo importa por um motivo específico: a projeção no plano do chão cruza o
raio da câmera com `z = chão`, então uma base a 1,5 m é posicionada **errada**
por melhor que seja vista. Girar não ajuda — o modelo de projeção é que está
errado para ela. O octomap mede onde a matéria está, em 3-D, e não tem esse
problema.

## Duas barreiras que nunca podem cair

Pousar fora de base é **eliminatório**.

1. **Uma perna de OLHAR nunca confirma nem pousa.** Chegar a um ponto de
   observação é o fim da viagem, não o começo de um pouso.
2. **Nada desce sem `target_id`.** `over pad None` é como o veículo termina no
   chão.

MEDIDO 2026-08-27, antes das duas: uma corrida que reportou "6 de 6 bases"
tinha pousado em **uma**. As outras cinco foram no meio da arena, sobre o que a
barriga topou ver.

E a busca esgotada **volta para a base de decolagem** — nunca pousa onde está.
Uma perna arriscada para uma base real ganha de um pouso certo no chão.

## A centragem na base

Durante a hover de confirmação o veículo se centraliza sobre o pad usando a
câmera de baixo. O mapeamento de pixel para metro **não é constante** — é uma
rotação, um sinal e uma escala, e nenhum dos três pode ser anotado:

- a escala muda com a altura, a cada frame de uma descida;
- a rotação depende de como a câmera foi parafusada, e a do drone real não é a
  do simulador;
- o sinal segue da rotação, e errá-lo não centraliza devagar — **afasta o
  veículo, acelerando**.

MEDIDO no simulador, 42 passos: `dv` acompanhou `body_x` a +0,810 e `du` não
acompanhou nada (0,00002 e 0,00014). Mesmo com a montagem escrita num arquivo
de configuração não há constante limpa.

Então `hydrone_nav/servo.py` **sonda**: duas sondagens ortogonais determinam a
jacobiana 2×2 exatamente, para qualquer montagem inversível. Broyden refina
depois. (Broyden sozinho, a partir de um palpite diagonal, **divergia para
1,66 m** numa montagem girada 90° — é para isso que as sondagens existem.)

**O que ele não consegue aprender:** para onde a câmera aponta com o veículo
nivelado — isso é o que "centralizado" significa. Numa fuselagem desalinhada,
centralizar no meio da imagem estaciona o drone fora da base pelo
desalinhamento × altura. É o parâmetro `pad_target_uv`, medido uma vez pairando
sobre uma base conhecida. **É o único número a calibrar no drone real.**

## O que ainda não está resolvido

- **Decolagem recusada em base elevada.** Depois de pousar a 0,89 m o FCU
  recusou `NAV_TAKEOFF` sem dar motivo. O livelock está consertado (aborta e
  diz), a causa não.
- **O VIO.** Tudo acima foi desenvolvido e medido com `--ground-truth`. Voando
  no VO o erro é de 7,83 m numa arena de 8 m. Ver `NAV-2026-08-27.md`.
