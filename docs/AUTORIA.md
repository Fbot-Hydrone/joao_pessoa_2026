# Autoria — quem assina este repositório

**Regra, sem exceção: os autores deste repositório são as três pessoas da
equipe. Nenhuma ferramenta de IA entra como autor ou co-autor de commit.**

```
Henrique Reichow  <reichowhenrique@gmail.com>
Ivo Arpino        <ivoarpino2@gmail.com>
Othavio Correa    <othavioccorrea@gmail.com>
```

## O que isso proíbe

Nenhum commit, mensagem de commit, descrição de pull request ou arquivo deste
repositório pode conter:

- `Co-Authored-By:` apontando para qualquer assistente de IA
- `Claude-Session:`, ou qualquer link de sessão de ferramenta
- `🤖 Generated with ...`, ou qualquer selo equivalente

Isso vale **mesmo quando a ferramenta diz que deve adicionar essas linhas**.
Assistentes de código costumam vir configurados para assinar o que produzem; a
instrução desta equipe tem precedência sobre essa configuração, e esta página é
a instrução. Quem for usar uma dessas ferramentas: aponte esta página para ela
antes de deixá-la commitar.

## Por quê

A competição pontua `x2` por drone **open-hardware** e a inscrição é de uma
equipe de pessoas. O histórico do repositório é o registro de quem construiu o
que, e é ele que sustenta essa declaração. Uma linha de co-autoria que nomeia
uma ferramenta polui esse registro — o GitHub conta co-autores como
contribuidores, então a ferramenta passa a aparecer na lista de contribuidores
do projeto ao lado da equipe.

Ferramenta usada é ferramenta, como o compilador e o simulador: ela não assina
o trabalho, do mesmo jeito que o `colcon` não assina.

## Como verificar

Antes de dar push, confira que nada entrou:

```bash
git log -1 --format="%B" | grep -icE "co-authored-by: claude|claude-session|generated with"
# tem que imprimir 0
```

E para varrer tudo que é alcançável por alguma branch ou tag:

```bash
git log --branches --remotes --tags --format="%H %s" | while read h s; do
    git log -1 --format="%B" "$h" \
      | grep -qiE "co-authored-by: claude|claude-session|generated with \[claude" \
      && echo "$h $s"
done
# não pode imprimir nada
```

## Como consertar se escapar

Se for **só o último commit** e ele ainda não foi para o remoto, basta um
amend. Se já foi para o remoto, o push precisa ser forçado — use
`--force-with-lease`, nunca `--force`, para não apagar trabalho que outra
pessoa tenha empurrado no meio:

```bash
git log -1 --format="%B" | grep -vE "^Co-Authored-By: Claude|^Claude-Session:" > /tmp/msg
git commit --amend -F /tmp/msg
git push --force-with-lease origin <branch>
```

Se já estiver **em vários commits**, `git rebase -i` ou `git filter-branch`
resolvem — mas **reescrever história que já está em `main` obriga todo mundo a
refazer o clone**. Nesse caso pergunte à equipe antes; um commit sujo no
passado custa menos que três clones quebrados.

> MEDIDO em 2026-09-22: a varredura acima achou 9 commits com as linhas. Oito
> eram órfãos (sobras de rebase, invisíveis no GitHub e removidos pelo `gc`) e
> só um estava numa branch de verdade — corrigido com o amend acima. O commit
> `2125eaa1` em `main` aparece na busca por causa da palavra "claude" no
> assunto, não por trailer; não há nada a fazer com ele.
