# EP2-Movobot

*Projet de Systèmes embarqués et robotique.*

## Installation

Le code comporte du code en **C** et en **Python**: les deux on une particulière intallation.
Si vous êtes sur *Windows 64bits* et vous souhaitez utiliser le programme **Python** à partir du fichier executable (*.exe*), sauter l'installation des packages **Python**.

### **C**

Les librairies suivantes doivent être placées dans */lib*:  

- e-puck2 main processor

### **Python**

Le code est testé sous [**Python3**](https://www.python.org/).

Les *packs Python* utilisés sont les suivants:

- [Matplotlib](https://matplotlib.org/)
- [Pyserial](https://www.python.org/)

#### **Comment installer les packages**

Ouvrer le terminal et entrez:

```bsh
pip install matplotlib
pip install pyserial
```

## Lancer le code

Connecter l'e-puck2 à votre ordinateur et en noter le port.

### **C**

Exporter le code dans l' [**e-puck2**](https://www.gctronic.com/doc/index.php/e-puck2) à partir d' [*Eclipse*](https://www.eclipse.org/).

### **Python**

Deux possibilités s'offre à vous en fonction de votre matériel.

1) Démarrer avec les scripts Python *(Compatible avec : Windows et Mac)*
    - Sur Mac

    ```bash
    python3 path_to_the_script/Scripts_python/ep2-movobot.py /dev/cu.usbmodemXXXX
    ```

    - Sur Windows

    ```cmd
    python path_to_the_script\Scripts_python\ep2-movobot.py comX
    ```

2) Démarrer avec l'éxécutable *(Compatible seulement avec Windows 64bits)*

    Écrire le port dans le fichier text *path_to_the_script\Scripts_python\precompiled_for_windows64bits\WRITE_PORT_HERE.txt*

    ```txt
    comX
    ```

    Puis lancer l'éxécutable *path_to_the_script\Scripts_python\precompiled_for_windows64bits\ep2-movobot.exe*.

Où *comX* et *cu.usbmodemXXXX* est le port auquel l'e-puck2 est connecté par bluetooth à votre ordinateur.

## Utilisation

3 état du robot sont possible lors de sa mise en marche:

- ### Control and read

    Permet de controler le robot avec les flèces du clavier. Les obstacles seront cartographié sur le graph au fur et à mesure du déplacement du robot.

- ### LiveIMU

    Permet de controler le robot avec les flèces du clavier. Additionnellement la valeur de l'IMU s'affiche en temps réel sur le côté. Les obstacles seront cartographié sur le graph au fur et à mesure du déplacement du robot.

- ### Plot calibration

    Il s'agit de calibrer les capteirs de distances. Une calibration par défaut est intégré mais il est **conseillé** de calibré l'e-puck2 à chaque nouvelle utilisation et de licker sur le bouton **Save** pour que cette calibration soit éffective.

- ### Reset Plot

    Ce bouton permet d'effacer toute référence aux obstacles rencontrés précédement et le graph est par la suite plus clair.

## Erreurs que vous pouvez rencontrer

Si il y a des erreurs d'exportation sur *Eclipse*, essayer de supprimer les fichiers:

- *libssp.a*
- *libssp_nonshared.a*

## Remerciments

Merci au cours de [Systèmes embarqués et robotique](https://edu.epfl.ch/coursebook/fr/systemes-embarques-et-robotique-MICRO-315) réalisé par le Prof. Mondada et au directeur des TP Dr. Burnier.
