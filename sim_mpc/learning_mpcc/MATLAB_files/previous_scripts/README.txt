-------------- Solver generation script --------------
Explicação breve de como gerar um novo solver.
Em caso de duvida: Gabriel Costa, gafc@live.com.pt
------------------------------------------------------

O script "main.m" deve ser executado para gerar um novo solver para uma determinada pista (ver opções do script).
O PC que gera o script deverá ter instalado:
- MATLAB
- FORCESPRO
- MATLAB Symbolic Toolbox
- MATLAB Optimization Toolbox

No final, o script terá gerado uma pasta com o nome "output_XXX" em que constará o solver e vários ficheiros necessários.
Esta pasta irá conter os seguintes ficheiros/subpastas:
- solverLMPC            (pasta)
- <nome_da_pista>.json  (ficheiro)
- Jac_p.cpp             (ficheiro)
- model_learning.yaml   (ficheiro)

-> a pasta "solverLMPC" contém o solver e deve ser substituida em control/learning_mpcc.
   Para além disto, as seguintes linhas de código deverão ser modificadas/adicionadas:
   - (modificar)   #include "solverLMPC/include/solverLMPC.h"   -->   #include "include/solverLMPC.h"
   - (adicionar)   #include "solverLMPC_casadi.c"   depois do   #include "solverLMPC_casadi.h"

-> o ficheiro "<nome_da_pista>.json" é necessário para o controlador saber onde estão os cones.
   Este ficheiro deverá ser adicionado em control/learning_mpcc/tracks
   e modificar para o respetivo nome da pista em control/learning_mpcc/launch/learning_mpcc.launch na linha 6

-> "Jac_p.cpp" é o ficheiro que contém a função do jacobiano da rede neuronal e deve ser adicionado/modificado em control/model_learning/src

-> "model_learning.yaml" contém os parâmetros iniciais da rede neuronal e deve ser adicionado/modificado em control/model_learning/config