# Implementação de Veículo de Carga Autônomo

## Introdução

Protótipo de veiculo autonomono  no LSE UFSC Joinville.


### Pré-requisitos

Precisa ter instalado no pc:

Ubuntu 18.04, ssh ativado, ros melodic, ros workspace ( https://catkin-tools.readthedocs.io/en/latest/# ), alguns pacotes do ros ( ainda serao listados).

Raspberry com Ubuntu Mate 18.04 (https://ubuntu-mate.org/download/), ros workspace ( usado o catkin_make por ser um ambiente com processamento limitado), vesao limita do ros melodic 18.04, nao é preciso instalar tudo, vou listar todos os pacotes necessarios.

Ter o arduino-ide (https://www.arduino.cc/en/main/software) e energia (https://energia.nu/) baixados.
```
Give examples
```

### Instalação

A step by step series of examples that tell you how to get a development env running

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

## Executando Programas
Entre no area de trabalho do ros (github_ws)
```
cd github_ws/

catkin clean -y

catkin build
```
Feche o terminal e abra um novo.


Controle pelo teclado

Terminal 1:

Inicializa o microcontrolador e os sensores(imu,encoders,ponte-h) no ros.
```
roslauch pegasus_bringup inicial.launch
```
Comunicação entre o teclado e as mensagens enviadar para o motor.
```
roslaunch pegasus_bringup keyboard_teleop.launch 
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

Add additional notes about how to deploy this on a live system

## Built With

* [Contributor Covenant](https://www.contributor-covenant.org/) - Used for the Code of Conduct
* [Creative Commons](https://creativecommons.org/) - Used to choose the license

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/PurpleBooth/a-good-readme-template/tags).


## License

This project is licensed under the [Attribution 4.0 International](LICENSE.md) Creative Commons License - see the [LICENSE.md](LICENSE.md) file for details



## Author

-   **Adriano**
-   **André**
-   **Jean**
-   **Lígia**
-   **Pablo Freitas Santos** - _Acesso ao Git_ - [PabloFreitasUfsc](https://github.com/PabloFreitasUfsc)


## Acknowledgments

-   <https://github.com/PurpleBooth>
