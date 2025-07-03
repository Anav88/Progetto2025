# SIMULAZIONE VOLO DI STORMI E PREDATORI

Cardillo Emanuele, Dipietro Alessandro, Stefanini Lorenzo

Data: 3 luglio 2025

- [Introduzione](README.md#introduzione)
- [Scelte progettuali ed implementative](README.md#scelte-progettuali-ed-implementative)
  - [Boid](README.md#boid)
  - [Predator](README.md#predator)
- [Istruzioni per eseguire il programma](README.md#istruzioni-per-eseguire-il-programma)
  - [Compilazione ed esecuzione](README.md#compilazione-ed-esecuzione)
- [Input](README.md#iniput)
- [Output](README.md#output)
- [Esempio di parametri ed analisi dei risultati](README.md#esempio-di-parametri-ed-analisi-dei-risultati)
- [Implementazione dei test](README.md#implementazione-dei-test)
- [Uso di sistemi di intelligenza artificiale](README.md#uso-di-sistemi-di-intelligenza-artificiale)

## Introduzione 
Il progetto simula il comportamento collettivo di uno stormo di uccelli (boid) e dei loro predatori, ispirandosi al modello di Craig Reynolds (1987). Il codice sorgente è disponibile su GitHub: https://github.com/Anav88/Progetto2025


## Scelte Progettuali ed Implementative 
Il progetto è strutturato in cinque file principali:
- Il file header - [`boids.hpp`](boids.hpp)
- Il file di implementazione - [`boids.cpp`](boids.cpp)
- Il file contenente i test - [`boids.test.cpp`](boids.test.cpp)
- Il main file - [`main.cpp`](main.cpp)
- Il file di lettura input - [`parameters.txt`](parameters.txt)

L'implementazione si basa su tre tipi aggregati principali:
- Struttura per vettori 2D - `Vec2f`:
  - Rappresenta posizioni e velocità nello spazio 2D;
  ``` cpp
  struct Vec2f {
    float x;
    float y;

    ...

    float angle() const; // calcola l'angolo formato con l'asse x
    float norm() const; // calcola la norma del vettore
  };

  ```
- Classe base - `Entity`:
  ```cpp
  class Entity {
    protected:
    Vec2f pos_;
    Vec2f vel_;

    public:
    ...

    void limit();
    //teletrasporta l'entità al lato opposto qualora fuoriesca dai bordi
  };
  ```
- Classi derivate - `Boid` e `Predator`:
  - Boid: 
    ```cpp
    class Boid : public Entity {
      private:
      Vec2f corr_vsep_{0.f, 0.f};
      Vec2f corr_vall_{0.f, 0.f};
      Vec2f corr_vcoes_{0.f, 0.f};
      Vec2f corr_vfuga_{0.f, 0.f};

      public:
      ...

      //Metodi per il calcolo delle velocità di correzione
      void vel_sep(Vec2f const &, float);
      void vel_all(Vec2f const &, float);
      void vel_coes(Vec2f const &, float);
      void vel_fuga(float, float);

      //Modifica posizione e velocità
      void correction();
      //Azzera le correzioni
      void reset_corr();
      //Modifica la velocità qualora la sua norma sia maggiore di quella consentita
      void vel_max();
    };

    ```
   
  - Predator:
    ```cpp
    class Predator : public Entity {
      private:
      Vec2f corr_vinseg_{0.f, 0.f};
      Vec2f corr_vsep_{0.f, 0.f};

      public:
      ...
  
      //Metodi per il calcolo delle due correzioni
      void vel_inseg(float);
      void vel_sep(float);

      //Modifica posizione e velocità
      void correction();
      //Azzera le correzioni
      void reset_corr();
    };

    ```
  

### Boid
Il comportamento dei boid è determinato da quattro regole, tre di queste descrivono le interazioni tra di essi e vengono applicate tenendo conto solo dei boid vicini, la quarta determina invece il rapporto con i predatori:
#### Regole
- **Separazione**: evita scontri e sovrapposizioni

```math
\vec{v}_{sep} = -s\sum_{j\ne i}(\vec{x}_{b_j}-\vec{x}_{b_i})\quad \text{se}\quad \left|\vec{x}_{b_i}-\vec{x}_{b_j}\right|\lt d_s
```

- **Allineamento**: uniforma la direzione di movimento complessiva

```math
\vec{v}_{all} = a(\frac{1}{n-1}\sum_{j\ne i}\vec{v}_{b_j} - \vec{v}_{b_i})
```

- **Coesione**: Attrae il boid verso il centro di massa dei vicini

```math
\vec{v}_{coes} = c(\vec{x}_{c}-\vec{x}_{b_i}) \quad \text{con}\quad \vec{x}_{c} = \frac{1}{n-1}\sum_{j\ne i}\vec{x}_{b_j}
``` 

- **Fuga**: induce i boid ad allontanarsi dai predatori

```math
|\vec{v}_{fuga}| = f
```

Dove, s, a, c, f sono dei parametri richiesti in input.

Ogni boid viene inizializzato con una posizione e una velocità generate casualmente, e successivamente aggiunto a un contenitore vettoriale che rappresenta lo stormo.
```cpp

void add_boid(std::vector<Boid> &boids_vector) {
  std::generate(boids_vector.begin(), boids_vector.end(),
                []() { return (Boid(rand_num())); });
}//dove rand_num è una funzione che genera una posizione e velocità casuali.

```
Per ogni elemento, le correzioni vengono calcolate dalla funzione `evaluate_boid_correction()`.
Questa chiama a sua volta una funzione specifica per il calcolo della correzione di fuga `evaluate_boid_corr_fuga()`. Inoltre, le sommatorie e gli n che compaiono nelle formule per le correzioni sono calcolati tramite l’algoritmo della standard library `std::accumulate()`. Per terminare chiama dei metodi della classe boid che terminano il calcolo delle correzioni.

```cpp

void evaluate_boid_correction(...){

  ...

  evaluate_boid_corr_fuga(...);

  ...
  auto result = std::accumulate(
        boids.begin(), boids.end(),
        ...,
        [&](...) {
          if (&boid_i != &boid_j) {
            //calcolo delle sommatorie e degli n
          }
          return acc;
        });

  ...
  boid.vel_sep(...);
  boid.vel_all(...);
  boid.vel_coes(...); 
  //metodi di Boid che completano il calcolo delle correzioni
}

```

Le nuove velocità e posizione sono dunque calcolate secondo le seguenti leggi:
```math

\vec{v}=\vec{v}_0+\vec{v}_{sep}+\vec{v}_{coes}+\vec{v}_{all}+v_{fuga} \\
\vec{x}=\vec{x}_0+\vec{v}*∆t

```

Il ∆t è stato definito dalla constexpr `TIME_STEP`, pari a 1/60 s. Tale scelta garantisce che ogni aggiornamento sia sincronizzato con il frame rate della finestra fissato a 60 FPS. Queste due operazioni sono eseguite dal metodo `correction()` della classe boid che però, prima di calcolare la nuova posizione, si accerta che la velocità sia nei limiti consentiti, e nel caso contrario riduce la norma della stessa mantenendo però costante la sua direzione.

### Predator
I predatori vengono creati al momento dell'interazione dell'utente tramite un click del mouse all'interno della finestra. Il programma limita la presenza a un massimo di cinque predatori: se questo numero è già stato raggiunto, ulteriori tentativi di creazione non produrranno alcun effetto.
#### Regole
Il suo comportamento è determinato da due regole:
- Inseguimento: L’agente individua il boid più vicino e lo insegue:
  ```math
  v_{fuga_x} = \text{VEL\_PRED\_INSEG} * cos(angle) \quad\quad v_{fuga_y} = \text{VEL\_PRED\_INSEG} * sin(angle)
  ```
  Dove angle è l'angolo formato con l'asse delle x dal vettore che collega il predatore con il boid individuato, mentre VEL_PRED_INSEG è una costante globale.

- Separazione: Mantiene predatori distanti:
  ```math
  v_{sep_x} = \text{VEL\_PRED\_SEP} * cos(angle) \quad\quad v_{sep_y} = \text{VEL\_PRED\_SEP} * sin(angle)
  ```
  Dove angle è l'angolo formato con l'asse delle x dal vettore che collega i due predatori, mentre VEL_PRED_SEP è una costante globale.


Queste due correzioni sono valutate dalla funzione `evaluate_pred_correction()`. Tramite l'algoritmo `std::min_element()` viene individuato il boid più vicino, e poi applicata la regola dell'inseguimento. Inoltre tale funzione valuta anche l’eventuale vicinanza di due predatori e in caso applica la seconda correzione.
Quando la distanza tra predatore e boid è inferiore a una soglia di tolleranza, il boid viene catturato. Tale operazione è effettuata dalla funzione `erase_boid()`.

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
### Compilazione ed esecuzione:
- Aprire il terminale e spostarsi nella cartella del progetto:
  ```bash
    $ cd /percorso/per/progetto2025 #inserire il percorso corretto
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
    $ build/Debug/main
  ```
- Eseguire i test:
  ```bash
   $ cmake --build build --config Debug --target test
   $ cmake --build build --config Release --target test  
  ```
## Input
All’interno della cartella progetto2025 è contenuto un file denominato [parameters.txt](parameters.txt). Esso contiene i seguenti parametri:
- Parametri di comportamento (valori float nell’intervallo [0,1]):
  - `s`, impiegato nel calcolo della velocità di separazione;
  -	`a`, impiegato nel calcolo della velocità di allineamento;
  -	`c`, impiegato nel calcolo della velocità di coesione.
- Distanze di interazione boid-boid, pred-pred (valori float positivi):
   - `d`, distanza di interazione per le regole di allineamento e coesione;
   - `d_s`, distanza per la separazione, con d_s < d;
   - `pred_dist_sep`, distanza per la separazione di due predatori.
- Numero di agenti (Valori int positivi):
  - `N`, numero totale di boid nella simulazione (consigliato: valore moderato).
- Parametri per la regola della velocità di fuga (valori float positivi):
  -	`boid_distance_fuga`, distanza entro la quale il boid si allontana dal predatore;
  -	`fact_fuga`, fattore della velocità di fuga del boid.

Se i parametri non rispettassero i vincoli richiesti, o ne dovesse mancare qualcuno, il programma termina con un messaggio d’errore.

## Output
Qualora l'input vada a buon fine l'output aspettato sarà il seguente:
- Finestra 600 x 600 con:
  - N Boid, Rappresentati da dei cerchi neri;
  - Predatori, Rappresentati da dei cerchi rossi (creati con un click del mouse)
- Statistiche: premere la barra spaziatrice per visualizzare velocità media e deviazione standard nel terminale.

## Esempio di Parametri ed Analisi dei Risultati
Un esempio raccomandato di parametri è fornito nel file [parameters.txt](parameters.txt). In tale configurazione, il parametro `c` assume un valore significativamente inferiore rispetto ai parametri `s` e `a`, per ridurre l'effetto di attrazione verso il centro di massa del gruppo. La variazione del parametro `d` influisce profondamente sulla dinamica del sistema: aumentando `d`, si osserva una diminuzione del numero di sottogruppi indipendenti, a favore di uno stormo più coeso.

Durante la simulazione, i boid tendono rapidamente a raggrupparsi e a stabilizzarsi in un apparente stato di quiete. Tuttavia, nonostante questo comportamento visivo, gli agenti mantengono una velocità vicina al massimo consentito. La velocità media del sistema si avvicina a zero, suggerendo un equilibrio nella distribuzione delle velocità. Tuttavia, l’elevata deviazione standard delle velocità individuali conferma che il sistema è dinamicamente attivo, con differenze significative nelle direzioni di moto.

L’implementazione della [regola di coesione](README.md#regole) ha evidenziato che, dividere per `n-1` porta a un comportamento anomalo dello stormo. Questo ha suggerito che il `-1` servisse a escludere il boid stesso dal calcolo del centro di massa. Dunque per la struttura del programma, che esclude autonomamente l'agente, è necessaria la divisione per `n`.

## Implementazione dei test
Per assicurare che la simulazione si comporti in modo corretto e coerente rispetto alle specifiche, è stata adottata una strategia di test, utilizzando il framework Doctest. L'obiettivo è individuare errori logici, verificare l’integrità dei dati e garantire che il comportamento emergente del sistema sia ragionevole anche in presenza di condizioni limite.

Sono dunque stati eseguiti i test su:
- Operazioni su `Vec2f`: operatori matematici e metodi `angle()`, `norm()`;
- Funzione `init_parametres()`: verifica il comportamento con valori fuori limite;
- Funzione `distance()`: tra boid e predatori;
- Metodi delle classi: `vel_max()` per Boid, `limit()` in entrambe le classi;
- Funzione `evaluate_boid_correction()`: testata sia con singole regole che con tutte attive;
- Comportamento dei predatori: inseguimento, separazione e fuga;
- Funzioni statistiche.

La combinazione di questi test costituisce una copertura ampia e ragionevole del sistema. Questa strategia permette di affermare che il comportamento della simulazione è stato verificato in modo sistematico e risulta **esente da errori evidenti**. Ciò nonostante non si può affermare la sua correttezza, bensì che i casi testati non hanno causato alcun errore.

## Uso di sistemi di Intelligenza Artificiale
L’intelligenza artificiale è stata utilizzata per generare porzioni di codice, in particolare:
- La funzione `Vec2f operator+= (Vec2f const &)`;
- La condizione dell’istruzione if alla riga 41/42 nel main:
  `if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space)`

È stata poi consultata per chiarimenti su metodi, algoritmi e per risolvere alcuni warning in fase di compilazione.
