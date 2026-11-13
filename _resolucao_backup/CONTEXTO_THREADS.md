# Contexto comum (Lab 2 UAVs, grupo 17) — ler antes de começar

Projeto: /home/caio/IST/ARob/LAB2 (não é repositório git; todas as threads partilham a mesma pasta, por isso
**só edites os ficheiros da tua tarefa**).
- Enunciado: enunciado/26_27_UAVs_Lab2.pdf (texto das perguntas já está em relatorio/main.tex). Plano e estado dos dados: PLANO_LAB2.md (LER).
- Dados: data/expA..D.mat (E ainda não existe). D só utilizável 5–18.4 s.
- Código: group17_lab2/ — `load_navdata.m` (descodifica navdata → t, theta, phi, psi, alt, acc[m/s²], gyr[rad/s], new),
  `lab2_datadir.m`, `savepdf.m`, `sec3_sensores.m` (já corre; gera figs/ e data/raw_incl.mat). `nd.new` = só amostras frescas (~77 Hz).
  Giros: gyr(:,1)=p (roll), gyr(:,2)=q (pitch). O inclinómetro do acelerómetro em voo correlaciona mal com o ângulo de bordo (força específica).
- MATLAB real funciona: `cd group17_lab2 && matlab -batch "script"` (R2025b, Control Toolbox disponível a verificar). Prefere MATLAB a Octave
  (o utilizador usa o Simulink, não corras Simulink). load_navdata é lento (~10 s por exp): cacheia em .mat se precisares.
- Relatório: relatorio/main.tex (coluna única, ≤15 páginas no total, TODAS as secções juntas — sê conciso: ~1–1.5 páginas por pergunta no máximo).
  Cada resposta vai no seu próprio ficheiro relatorio/answers/qX_Y.tex (já incluído via \input dentro de \begin{answer}). Escreve LaTeX
  (ambiente já dentro da caixa answer; o `\todoanswer` é placeholder — substitui-o). Figuras em relatorio/figs/ (PDF, copiadas de
  group17_lab2/figs/). Não edites main.tex. Não há pdflatex local: confere a sintaxe com cuidado (ou `nix shell nixpkgs#texliveSmall -c pdflatex` se quiseres compilar um teste fora de relatorio/).
- Estilo das respostas: inglês, rigoroso, com números reais dos dados e interpretação física; mostra equações.
- Não faças git commit. Não mexas nos ficheiros .slx. Termina com um resumo curto: ficheiros criados, números-chave, o que ficou por fazer.
