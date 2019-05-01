# Smart-Office-Illumination-System

Sistema de controlo de iluminação de uma sala que contém 127 iluminárias, onde existe comunicação entre todas as iluminárias (representadas através de um arduino), de modo a que o consumo de energia do sistema global seja o mínimo possível. A iluminação de uma iluminária influência as iluminárias à sua volta, pelo que se o utilizador de uma iluminária deseja ter 10 Lux, esta pode não ter de fornecer 10 Lux, pois tem de se ter em atenção o efeito de todas as outras, assim como as perturbações externas (janela aberta por exemplo). Para a resolução do problema é realizado o algoritmo consensus para minimizar o custo de energia do sistema distribuído descentralizado.
- Além disto, existe a funcionalidade administrador que permite controlar tudo e recolher dados estatísticos de toda a sala (realizado com um RASPBERRY PI).
- Comunicação dos arduinos e do RASPBERRY feita através de I2C.

# Grade:
Parte1: 19/20
Parte2: 20/20
