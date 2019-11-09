# Implementação de Veículo de Carga Autônomo

## Introdução

Protótipo de veiculo autonomono  no LSE UFSC Joinville.


### Pré-requisitos

Precisa ter instalado no pc:

Ubuntu 18.04, ssh ativado, ros melodic, ros workspace ( https://catkin-tools.readthedocs.io/en/latest/# ), alguns pacotes do ros ( ainda serao listados).

Raspberry com Ubuntu Mate 18.04 (https://ubuntu-mate.org/download/), ros workspace ( usado o catkin_make por ser um ambiente com processamento limitado), vesao limita do ros melodic 18.04, nao é preciso instalar tudo, vou listar todos os pacotes necessarios.

Ter o arduino-ide (https://www.arduino.cc/en/main/software) e energia (https://energia.nu/) baixados.

### Instalação

Primeiramento no pc(monitorador), entre na area de trabalho do ros (github_ws):
```
cd github_ws/
```

Baixe a ultima versão do código fonte:
```
git clone https://github.com/PabloFreitasUfsc/AMR_UFSC
```

Limpe o workspace:
```
catkin clean -y

```
 Re-construa o espaço:
```
catkin build
```
Feche o terminal e abra um novo toda vez que recompilar os códigos.


## Controle pelo teclado

Inicializa o microcontrolador e os sensores(imu,encoders,ponte-h) no ros:
```
roslauch pegasus_bringup inicial.launch
```

Comunicação entre o teclado e as mensagens enviadar para o motor:
```
roslaunch pegasus_bringup keyboard_teleop.launch 
```
## Mapeamento

Inicializa o microcontrolador e os sensores(imu,encoders,ponte-h) no ros:
```
roslauch pegasus_bringup inicial.launch
```

Ativando o lidar
```
roslaunch pegasus_bringup lidar.launch
```
Carregando o algoritmo SLAM ( Simultaneas Localization and Mapping), nesse caso sera usado o gmapping:

```
roslaunch pegasus_bringup mapeamento_demo.launch 
```
Abrir um arquivo Rviz para vizualiação da criação do mapa:
```
roslaunch pegasus_bringup mobot_rviz_navegacao.launch
```
Por fim para salvar o mapa feito, onde o argumento ~/github_ws/src/AMR_UFSC/pegasus_bringup/map/test_map2 pode ser trocado pelo diretório e nome do arquivo que queira salvar:
```
rosrun map_server map_saver -f ~/github_ws/src/AMR_UFSC/pegasus_bringup/map/test_map2
```

## Navegação


Inicializa o microcontrolador e os sensores(imu,encoders,ponte-h) no ros:
```
roslauch pegasus_bringup inicial.launch
```

Ativando o lidar
```
roslaunch pegasus_bringup lidar.launch
```
Carrega o mapa que o robô precisará navegar:
```
roslaunch pegasus_bringup amcl_demo.launch 
```
Abrir um arquivo Rviz para vizualiação da navegação:
```
roslaunch pegasus_bringup mobot_rviz_navegacao.launch
```

### Testes

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Notas de Desenvolvimento

Link para a documentação: https://drive.google.com/open?id=1Uiy5zNxnDLPrxDTVcyMnjF-LcmeB3GVd0uSXu1x8qcg



## Author

-   **Adriano**
-   **André**
-   **Jean**
-   **Lígia**
-   **Lucas Dallamico**
-   **Pablo Freitas Santos** - _Acesso ao Git_ - [PabloFreitasUfsc](https://github.com/PabloFreitasUfsc)


## Acknowledgments

-   <https://github.com/PurpleBooth>
-    https://catkin-tools.readthedocs.io/en/latest/#
