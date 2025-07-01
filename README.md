# SIMULAZIONE VOLO DI STORMI E PREDATORI
Il progetto è stato sviluppato utilizzando Git ed è disponibile alla seguente repository: https://github.com/Anav88/Progetto2025

Il progetto simula il comportamento collettivo di uno stormo di uccelli (boid) e dei loro predatori, ispirandosi al modello di Craig Reynolds (1987). 

## SCELTE PROGETTUALI ED IMPLEMENTATIVE
Il progetto è strutturato in quattro file principali:
- L’header file - [boids.hpp](boids.hpp)
- Il file di implementazione - [boids.cpp](boids.cpp)
- Il file contenente i test - [boids.test.cpp](boids.test.cpp)
- Il main file - [main.cpp](main.cpp)

L'implementazione si basa su tre tipi aggregati principali:
- Struttura per vettori 2D - Vec2f:
  - Rappresenta posizioni e velocità nello spazio 2D;
  - Contiene metodi per il calcolo della norma e della direzione;
- Classe base - Entity:
  - Contiene le proprietà base di ogni entità simulata:
    - Due variabili Vec2f posizione e velocità;
    - Metodi quale limit() per gestire i bordi dello spazio simulato;
- Classi derivate - Boid e Predator:
  - Esse contengono variabili e metodi specifici che caratterizzano l'entità.

## Boid
Il comportamento dei boid è determinato da quattro regole, tre di queste descrivono le interazioni tra di essi e vengono applicate tenendo conto solo dei boid vicini, la quarta determina invece il rapporto con i predatori:
### Regole
- **Separazione**: evita scontri e sovrapposizioni
```math
\vec{v}_{sep} = -s\sum_{j\ne i}(\vec{x}_{b_j}-\vec{x}_{b_i})\quad \text{se}\quad \left|\vec{x}_{b_i}-\vec{x}_{b_j}\right|\lt d_s
```
- ****Allineamento**: uniforma la direzione di movimento complessiva
```math
\vec{v}_{all} = a(\frac{1}{n-1}\sum_{j\ne i}\vec{v}_{b_j} - \vec{v}_{b_i})
```
- **Coesione**: Attrae il boid verso il centro di massa dei vicini
```math
\vec{v}_{coes} = c(\vec{x}_{c}-\vec{x}_{b_i}) \quad \text{con}\quad \vec{x}_{c} = \frac{1}{n-1}\sum_{j\ne i}\vec{x}_{b_j}
``` 
- **Fuga**: induce i boid ad allontanarsi dai predatori.
```math
|\vec{v}_{fuga}| = f
```
Dove, s, a, c, f sono dei parametri richiesti in input.

Ciascun boid è inizializzato con posizione e velocità casuali, ed è inserito in un vettore.
Per ogni elemento, le correzioni vengono calcolate dalla funzione evaluate_boid_correction().
Questa chiama a sua volta una funzione specifica per il calcolo della correzione di fuga - evaluate_boid_corr_fuga(), ed inoltre, tramite l’algoritmo della standard library std::accumulate(), calcola le sommatorie e gli n che compaiono nelle formule per le correzioni. Per terminare chiama dei metodi della classe boid che terminano il calcolo delle correzioni.

Le nuove velocità e posizione sono dunque calcolate secondo le seguenti leggi:
```math
\vec{v}=\vec{v}_0+\vec{v}_{sep}+\vec{v}_{coes}+\vec{v}_{all}+v_{fuga}\\
\vec{x}=\vec{x}_0+\vec{v}*∆t
```

Il ∆t è stato definito tramite constexpr ed è pari a 1/60 s. Tale valore è stato scelto affinché ogni correzione fosse coerente con il “refresh” della finestra generata da SFML che avviene proprio ogni sessantesimo di secondo. Queste due operazioni sono eseguite dal metodo correction() della classe boid che però, prima di calcolare la nuova posizione, si accerta che la velocità sia nei limiti consentiti, e nel caso contrario riduce la norma della stessa mantenendo però costante la sua direzione.

## Predator
I predatori non sono creati all'inizio della simulazione ma solo nel caso in cui l'utente interagisca facendo un click del mouse all'interno della finestra. Il programma impedisce la presenza di più di 5 preddatori, dunque, qualora si fosse già raggiunto il numero consentito e si provasse nuovamente ad eseguire la procedura per formare un nuovo agente, non accadrebbe nulla. 
### Regole
Il suo comportamento è determinato da due regole:
- Inseguimento: L’agente individua il boid più vicino e lo insegue
  ```math
  v_{fuga_x} = VEL\_PRED\_INSEG * cos(angle) \quad\quad v_{fuga_y} = VEL\_PRED\_INSEG * sin(angle)
  ```
  dove angle è l'angolo formato con l'asse delle x dal vettore che collega il predatore con il boid individuato.

- Separazione: Mantiene predatori separati
  ```math
  v_{sep_x} = VEL\_PRED\_SEP * cos(angle) \quad\quad v_{sep_y} = VEL\_PRED\_SEP * sin(angle)
  ```
  dove angle è l'angolo formato con l'asse delle x dal vettore che collega i due predatori.


Queste due correzioni sono valutate dalla funzione - evaluate_pred_correction(). L’algoritmo std::min_element() individua il boid più vicino così da poter applicare la regola dell'inseguimento. Inoltre tale funzione valuta anche l’eventuale vicinanza di due predatori. Nel caso di due entità vicine è applicata la seconda correzione. Anche in questo caso il modulo di tale velocità è definito dalla costante VEL_PRED_SEP, e sono modificate le sole componenti.
Nel momento in cui le posizioni del predatore e della preda sono uguali, a meno di un piccolo errore, il boid viene “mangiato”. Tale operazione è effettuata dalla funzione - erase_boid().

## Istruzioni per eseguire il programma
Per eseguire il programma è necessario avere installato CMake, Ninja, e la libreria esterna SFML. Qualora tali componenti non fossero stati precedentemente installati, ecco i passaggi necessari:
- MacOS
  - Aprire il terminale e installare tali componenti mediante brew, grazie al seguente
  comando:
    ```bash
    $ brew install cmake ninja sfml@2
    ```
  - A seguito dell’operazione è possibile verificare la corretta installazione delle componenti tramite i seguenti comandi:
    ```bash
    $ brew install cmake ninja sfml@2
    ```
- Linux
  - Aprire il terminale ed eseguire il seguente comando:
    ```bash
    $ sudo apt install cmake ninja-build libsfml-dev
    ```
Compilazione ed esecuzione:
- Aprire il terminale e spostarsi nella cartella del progetto:
  ```bash
    $ cd /percorso/progetto2025
  ```
- Configurare l’ambiente di build con il comando:
  ```bash
    $ cmake -S . -B build -G "Ninja Multi-Config"
  ```
  Questo comando, grazie al file CMakeLists.txt presente all’interno della cartella, genera e configura una cartella di compilazione denominata build.
- Compilare il progetto:
  ```bash
    $ cmake --build build --config Debug
    $ cmake --build build --config Release
  ```
- Eseguire il programma principale:
  ```bash
    $ cmake --build build --config Debug --target main
  ```
- Eseguire i test:
  ```bash
   $ cmake --build build --config Debug --target test
   $ cmake --build build --config Release --target test  
  ```
## Input Output
All’interno della cartella progetto2025 è contenuto un file denominato [parameters.txt](parameters.txt). Esso contiene i seguenti parametri:
- Parametri di comportamento (valori float nell’intervallo [0,1]):
  - s, impiegato nel calcolo della velocità di separazione;
  -	a, impiegato nel calcolo della velocità di allineamento;
  -	c, impiegato nel calcolo della velocità di coesione.
- Distanze di interazione boid-boid, pred-pred (valori float positivi):
   - d, distanza di interazione per le regole di allineamento e coesione;
   - d_s, distanza per la separazione, con d_s < d;
   - pred_dist_sep, distanza per la separazione di due predatori.
- Numero di agenti (Valori int positivi):
  - N, numero totale di boid nella simulazione (consigliato: valore moderato).
- Parametri per la regola della velocità di fuga (valori float positivi):
  -	boid_distance_fuga, distanza entro la quale il boid scappa dal predatore;
  -	fact_fuga, fattore della velocità di fuga del boid.

Se i parametri non rispettassero i vincoli richiesti, o ne dovesse mancare qualcuno, il programma termina con un messaggio d’errore. Al contrario il programma apre una finestra 600 x 600 con N boid, rappresentati con dei cerchi neri. Un click del mouse può inoltre far comparire un predatore, raffigurato da un cerchio rosso, nel punto dove è avvenuto l’evento.
Inoltre, durante l’esecuzione, premendo la barra spaziatrice, si possono visualizzare sul terminale la velocità media e deviazione standard dei boid.

## Implementazione dei test
Per validare il comportamento del sistema di simulazione dello stormo, sono
stati implementati test automatici usando il framework Doctest.
I test verificano:
- Operazioni su Vec2f: operatori matematici e metodi angle(), norm();
- Funzione init parametres(): verifica il comportamento con valori fuori limite;
- Funzione distance(): tra boid e predatori;
- Metodi delle classi: vel max() per Boid, limit() in entrambe le classi;
- Funzione evaluate boid correction(): testata sia con singole regole che con tutte attive;
- Comportamento dei predatori: inseguimento, separazione e fuga;
- Funzioni statistiche.

I test sono stati spesso valutati anche in casi limite (es. eccezioni), per rivelare
eventuali errori.

## Uso di sistemi di Intelligenza Artificiale
L’intelligenza artificiale è stata utilizzata per generare porzioni di codice, in particolare:
- La funzione Vec2f &operator+= (Vec2f const &);
- La condizione dell’istruzione if alla riga 41/42 nel main:
  if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space)

È stata poi consultata per chiarimenti su metodi, algoritmi e per risolvere alcuni warning in fase di compilazione.








