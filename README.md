# Implementação de Semáforo e Problema Produtor X Consumidor

Modificar

## Introdução

Problema 1- Considere ainda que existem 6 chefs e 1 atendente. Um chef leva 50 μsec para produzir uma refeição e o atendente gasta 10 μsec para entregar a refeição na mesa.

Problema 2 - Considere ainda que existem 1 chefs e 6 atendente. Um chef leva 50 μsec para produzir uma refeição e o atendente gasta 15 μsec para entregar a refeição na mesa.

Número máximo de clientes a serem atendidos é 200. No final apresentará uma saida para identificar o mais e menos ocioso.

## Executando Programas

Exemplo de teste para o N-1

    pablo@pablo-X510UR:~/Documents/Drive/UFSC/5fase/Sistemas-Operacionais/Atividades5 - Programação Concorrente/N_1$ ls
    bib.hpp  main.cpp  Makefile  README.md  semaforo.cpp
    pablo@pablo-X510UR:~/Documents/Drive/UFSC/5fase/Sistemas-Operacionais/Atividades5 - Programação Concorrente/N_1$ make
    g++ *.cpp -o prototype -g -lpthread -std=c++11;
    #gcc *.c -o Prototype -g -lpthread;
    pablo@pablo-X510UR:~/Documents/Drive/UFSC/5fase/Sistemas-Operacionais/Atividades5 - Programação Concorrente/N_1$ ./prototype

    Rodando 6 Chefs em paralelo e 1 Atendente

    Chef - 0 produziu: 16 / 200 pratos.
    Chef - 1 produziu: 42 / 200 pratos.
    Chef - 2 produziu: 25 / 200 pratos.
    Chef - 3 produziu: 35 / 200 pratos.
    Chef - 4 produziu: 67 / 200 pratos.
    Chef - 5 produziu: 15 / 200 pratos.

    Chef mais ocioso: 5
    Chef menos ocioso: 4

Exemplo de teste para o N-1

    pablo@pablo-X510UR:~/Documents/Drive/UFSC/5fase/Sistemas-Operacionais/Atividades5 - Programação Concorrente/1_N$ ./prototype

    Rodando 6 Atendentes em paralelo e 1 Chef

    Atendente - 0 entregou: 51 / 200 pratos.
    Atendente - 1 entregou: 40 / 200 pratos.
    Atendente - 2 entregou: 26 / 200 pratos.
    Atendente - 3 entregou: 38 / 200 pratos.
    Atendente - 4 entregou: 15 / 200 pratos.
    Atendente - 5 entregou: 29 / 200 pratos.

    Atendente mais ocioso: 4
    Atendente menos ocioso: 0

## Author

-   **Pablo Freitas Santos** - _Acesso ao Git_ - [PabloFreitasUfsc](https://github.com/PabloFreitasUfsc)

## Acknowledgments

-   <https://github.com/PurpleBooth>
