# A mesma missão em sete arenas — e o defeito que só a sétima revelaria

Escrito em 2026-09-02. Registra a primeira varredura por seeds do `map_sweep`, o
defeito que ela encontrou, e o que continua aberto.

    scripts/seed_sweep.sh 1 2 3 4 5 6
    MISSION=--phase1 scripts/seed_sweep.sh 1 2 3 4 5 6    # o U, para comparar

---

## 1. Por que varrer seeds

As duas missões tinham sido medidas em **uma** arena, seed 10, e ajustadas
olhando para ela. Um limiar que por acaso serve a seis posições específicas de
base é indistinguível de um que generaliza — até ser voado em arranjos que
ninguém olhou.

A verdade é **gerada, não anotada**: `sample_bases(seed)` é a mesma função que o
simulador chama para posicionar as bases, então para uma seed dada ela
reproduz a arena exatamente. E um pouso só conta como **pouso em base** quando a
altura de repouso bate o topo dela (topo + 0,13 m, assinatura medida) — a missão
não sabe distinguir base de chão ao lado.

## 2. O que saiu

| seed | detectou | pousos | válidos | erro do mapa | desfecho |
|---|---|---|---|---|---|
| 4 | 6/6 | 6 | **6** | 0,31 m | completa |
| 10 | 6/6 | 5 | 5 | 0,45 m | abortada |
| 1 | 5/6 | 6 | 4 | 0,35 m | completa |
| 2 | 5/6 | 6 | 5 | 0,47 m | completa |
| 6 | 5/6 | 5 | 5 | 0,38 m | completa |
| 3 | 3/6 | 3 | **0** | 0,35 m | abortada |
| 5 | 3/6 | 3 | 2 | 0,44 m | completa |

## 3. O defeito, e ele é monotônico

Ordenando por quantas bases a arena põe **no telhado da casinha** (1,5 m do
chão, `0,80` no frame da missão):

| bases no telhado | seeds | detectadas |
|---|---|---|
| nenhuma | 4, 10 | **6/6**, 6/6 |
| uma | 1, 2, 6 | **5/6**, 5/6, 5/6 |
| duas | 3, 5 | **3/6**, 3/6 |

**Sete arenas, zero exceção.**

A causa é aritmética. O espaçamento das faixas vem da pegada da câmera — essa
parte era o ponto do desenho e está certa. Mas `_sweep_swath_m` media a altura
até o **chão**:

    sobre o chão:     3,2 m de altura  ->  pegada 4,80 m  ->  faixas a 3,60 m
    sobre o telhado:  1,7 m de altura  ->  pegada 2,55 m

As faixas ficavam **3,60 m** apartadas enquanto a câmera cobria **2,55 m** —
mais de um metro sem varredura nenhuma, exatamente sobre a única estrutura que
carrega base elevada.

Corrigido com `sweep_max_surface_m` (1,5 m, que é o número da própria
competição para o telhado e para a base mais alta): a altura passa a ser medida
sobre a superfície mais alta que a varredura sobrevoa. A arena passa de 3 para
5 faixas.

**Por que a seed 10 escondia isso:** ela não tem nenhuma base no telhado. Era a
única arena em que o modo havia sido medido.

### E a correção NÃO resolveu o problema do telhado

Isto precisa ficar registrado com a mesma clareza do achado, porque a narrativa
era boa demais. As seeds 3 e 5 foram revoadas com a correção:

| seed | | nível 2 voado? | detectadas | pousos válidos |
|---|---|---|---|---|
| 3 | antes | **não** | 3/6 | 0 de 3 |
| 3 | depois | **não** | 4/6 | 4 de 6 |
| 5 | antes | sim, 3 faixas @ 4,80 m | 3/6 | 2 de 3 |
| 5 | depois | sim, **5 faixas @ 2,55 m** | 3/6 | 2 de 3 |

**A seed 3 nunca chegou ao nível 2 em nenhuma das duas corridas** — ela pousa
durante o perímetro e o `land_during_survey` consome a missão antes das faixas.
A correção é inerte para ela, então a melhora de 0 para 4 pousos válidos é
**variação entre corridas, não efeito do conserto**.

**A seed 5 é o único teste controlado**, e nela a correção fez exatamente o que
promete — 3 faixas viram 5, o swath cai de 4,80 para 2,55 m — e o resultado
**não muda**.

Conclusão honesta: o buraco de cobertura era aritmética real e a correlação
monotônica em sete arenas era forte, mas a cobertura **não era o que custava as
bases do telhado**. A correção fica porque a aritmética dela está certa; ela
apenas não é o conserto que se procurava. E n=1 em teste controlado é fraco nos
dois sentidos.

### O que o telhado parece ser, então

Na seed 3 revoada, a base 5 (telhado) FOI detectada e posicionada **1,01 m**
errado:

    pad 1 (-1.96, +1.71)  ->  base 5 (-2.95, +1.50)   d = 1.01 m

Isso é projeção, não varredura. A hipótese que se encaixa: o octomap não tem o
topo da casinha — a ZED aponta para a frente e o perímetro voa as bordas, então
o telhado a 0,80 pode nunca entrar na banda de profundidade. Sem resposta do
mapa, a projeção cai no rangefinder, que mede o nadir do VEÍCULO; com o pad na
borda do quadro isso desloca cerca de 1 m, que é a assinatura observada.

**Como confirmar sem voar de novo:** contar o campo `source` das detecções
sobre a casinha (3 = mapa, 1 = profundidade/rangefinder). Não feito.

## 4. O segundo achado: a missão declara pousos que não aconteceram

Na seed 3, os três pousos registraram `-0.58`, `-0.58` e `-0.36`. O chão está em
`-0,70`, e repouso no chão dá `-0,70 + 0,13 = -0,57`.

**Dois pousos foram no piso nu, e a missão contou os três como sucesso.**

E as entradas do mapa explicam como ele chegou lá — três **fantasmas** a mais de
um metro de qualquer base real:

    pad 1 (-1.64, +1.55)   base mais proxima a 1,09 m
    pad 2 (-1.81, -0.54)   base mais proxima a 1,51 m
    pad 3 (-1.40, -2.13)   base mais proxima a 1,13 m

A missão voou até eles, a barriga "confirmou", e ele desceu em chão vazio.

Isto **não está corrigido**, e é a mesma raiz do erro de fusão de 0,31–0,47 m
que aparece em todas as sete arenas: a detecção individual mede 6 cm e o mapa
mede 45. Nas arenas boas a cauda do erro ainda cai dentro da base de 1 m e a
centragem visual salva o pouso; na seed 3 a cauda esticou até 1,5 m e não salvou.

## 5. O que isto diz sobre o que fazer

O gargalo **mudou de lugar**. Não é mais detecção — a projeção pelo mapa entrega
6 cm por leitura. É a **fusão do `pad_map`**, que dilui isso para 45 cm e
ocasionalmente produz fantasmas que viram pouso em chão vazio.

Três coisas em ordem de retorno, nenhuma feita:

1. **Verificação de altura no pouso.** Os dados existem (`pad.height_measured`,
   e a assinatura de 0,12–0,14 m) e nada compara. É o que impede um pouso
   inválido de contar como sucesso, e é barato.
2. **A fusão.** `pad_map` pondera por `confiança / alcance²`, e num corta-grama
   2,0 m contra 3,9 m não separa o suficiente — as poucas leituras de nadir se
   perdem entre muitas medianas de meio-de-faixa. Vale investigar o viés
   sistemático em −y antes de mexer na ponderação.
3. **O frame da octree no planejador.** `phase1_mission_node` consulta o mapa
   em coordenadas de `map` contra uma árvore em `odom`. Ver
   [`MAP-SWEEP-2026-09-02.md`](MAP-SWEEP-2026-09-02.md) §3.

## Relacionados

- [`MAP-SWEEP-2026-09-02.md`](MAP-SWEEP-2026-09-02.md) — o modo, e como ele funciona
- [`NAV-E-DETECCAO-2026-09-02.md`](NAV-E-DETECCAO-2026-09-02.md) — a detecção da ZED
- `scripts/seed_sweep.sh`, `scripts/score_run.py` — o arnês
