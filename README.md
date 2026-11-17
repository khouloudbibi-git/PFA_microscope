# PFA_Microscope – Firmware STM32CubeIDE

Firmware de l'automatisation du plateau du microscope Clever C320 (carte NUCLEO-F411RE).

## Ouvrir le projet
STM32CubeIDE → **File > Import… > General > Existing Projects into Workspace** → sélectionner ce dossier → **Build** (marteau) → **Run/Debug** (ST-LINK).

## Brochage
| Fonction | Broche | Périphérique |
|---|---|---|
| X STEP / DIR / EN | PA0 / PA1 / PB0 | GPIO |
| Y STEP / DIR / EN | PA6 / PA7 / PB1 | GPIO |
| Z STEP / DIR / EN | PB6 / PB7 / PB8 | GPIO |
| Encodeur X (A/B) | PA15 / PB3 | TIM2 mode encodeur |
| Encodeur Y (A/B) | PA8 / PA9 | TIM1 mode encodeur |
| Encodeur Z (A/B) | PB4 / PB5 | TIM3 mode encodeur |
| Fin de course X / Y1 / Y2 | PC0 / PC1 / PC2 | EXTI, pull-up, actif bas |
| NTC 1 / NTC 2 | PA4 / PA5 | ADC1 IN4 / IN5 |
| PWM verre ITO | PB9 | TIM4 CH4, 1 kHz |
| PC | PA2 / PA3 | USART2 115200 8N1 (COM virtuel ST-LINK) |
| Base de temps des pas | – | TIM5, 40 kHz |

## Modules
- `Core/Src/motion.c` : génération des pas (rampe trapézoïdale), encodeurs, fins de course, homing, boucle fermée.
- `Core/Src/thermal.c` : acquisition NTC, PID 37 °C, PWM ITO, sécurités (coupure > 45 °C, watchdog analogique ADC).
- `Core/Src/comm.c` : protocole texte avec le pont Python (liste des commandes en tête du fichier).
- `Core/Inc/app_config.h` : **tous les paramètres à calibrer** (pas de vis, micro-pas, sens, PID, Beta NTC…).

## Exemple de session série
```
> PING                 < PONG
> MOTORS ON            < OK MOTORS ON
> HOME                 < OK HOME ... EVT HOME X OK / EVT HOME Y OK / EVT HOME Z OK
> MOVE X 150.5         < OK MOVE X ... EVT DONE X
> GOTO Z -20           < OK GOTO Z
> TSET 37              < OK TSET
> HEAT ON              < OK HEAT ON
< STA X=150.5 Y=0.0 Z=-20.0 EX=150.4 ... T=36.92 SP=37.00 PWR=41 HEAT=1 TST=HEATING ...
```

## Points à vérifier sur le prototype
1. **Horloge** : HSE 8 MHz en mode *Bypass* (MCO du ST-LINK de la Nucleo) → 84 MHz. Si un quartz 25 MHz est monté, repasser en *Crystal* avec PLLM = 25 dans le `.ioc`.
2. **PA5 = LED LD2 de la Nucleo** : la LED charge le pont diviseur de la NTC 2. Retirer le pont SB21 ou déplacer la NTC 2.
3. **Pas de vis / micro-pas / sens moteurs et encodeurs** : `app_config.h`. Désactiver la boucle fermée (`CL OFF`) tant que le sens des encodeurs n'a pas été vérifié.
4. **Gains PID** : à régler sur le plateau réel (commande `PID kp ki kd`).
