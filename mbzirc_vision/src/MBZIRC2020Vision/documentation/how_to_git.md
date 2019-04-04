# BREVE GUIDA ALL'USO DI GIT (it)

=========================================

## Cosa sono git e github?

git è un sistema software CVS, ovvero di controllo versione. In parole semplici, git tiene traccia dei cambiamenti dei file all'interno di un progetto quando questi subiscono modifiche.
github è un sito di hosting per progetti che usano git, così come lo sono bitbucket o gitlab.
git è la tecnologia, github un'implementazione.

### Vantaggi

Usare git(hub) permette di:
* avere sempre un backup del progetto
* tener traccia di tutte le modifiche avvenute durante lo sviluppo
* far lavorare più persone contemporaneamente al progetto
* provare nuove funzionalità mantenendo in parallelo le versioni funzionanti ('branch')

## Come funziona git

Git è uno strumento avanzato ed ha tanti meccanismi complessi, ma ai fini di MBZIRC bastano le funzionalità più semplici.  
Su github c'è il repository del progetto, la cartella "centrale". Ciascun collaboratore inizialmente scarica una copia completa del progetto sul suo pc (in gergo 'clone'). A quel punto può iniziare a modificare i file, poi quando è soddisfatto salva le modifiche ('commit') ed eventualmente le carica sul repository centrale ('push').
La volta successiva che vuole lavorare al progetto, l'utente dovrà ricordarsi di fare all'inizio un 'pull' per scaricare da github le eventuali modifiche recenti compiute dai suoi collaboratori.
In qualsiasi momento è possibile ritornare ad un commit precedente, ovvero una precedente versione.


## Istruzioni principali

Le seguenti istruzioni costituiscono il nucleo indispensabile di comandi da sapere per collaborare al progetto di MBZIRC attraverso git.
Per una comprensione più completa e dettagliata ci sono tante guide online, per esempio:  
- https://try.github.io/
- https://git-scm.com/docs/gittutorial
- https://it.atlassian.com/git/tutorials

### Cose da fare solo la prima volta (su una nuova postazione)

1) Posizionarsi nella cartella in cui mettere il progetto e scaricarlo con:  
   ```git clone https://github.com/leoll2/MBZIRC20_Code.git```

2) Verificare che effettivamente i file sono stati scaricati

3) Configurare la propria identità (nome e mail):  
   ```git config user.name "Mario Rossi"```  
   ```git config user.email pippo@pluto.com```
   
   
### Da fare ogni volta che si lavora 
   
4) Verificare in che branch ci si trova (altrimenti si rischia di modificare i file sbagliati):  
   ```git branch```  
   Appare la lista dei branch esistenti ed evidenziato in verde quello attuale.
   
5) Se il branch è quello sbagliato, spostarsi su quello giusto:  
   ```git checkout <nome_branch>```  
   Poi rifare `git branch` per avere conferma.
 
6) Verificare che non ci siano modifiche non "committate" dall'ultima volta. Se ci sono, fare il commit (vedi sotto) o chiedere aiuto.  
   ```git status```
    
7) Sincronizzare i file locali con quelli aggiornati su github (pull). Questo step è FONDAMENTALE per non lavorare su versioni vecchie ed avere dopo noiosi problemi di merge (vedi sotto).
    ```git pull```
    
8) Fare le proprie modifiche ai file.

9) Controllare quali file sono stati modificati (in rosso):  
   ```git status```
   Per visualizzare anche le modifiche stesse (opzionale):  
   ```git diff```
   
10) Specificare i file che andranno nel prossimo commit (spesso tutti quelli rossi):  
    ```git add <nomi file>```
    Oppure per aggiungerli tutti:  
    ```git add -A```

11) Controllare che siano stati aggiunti (in verde):  
    ```git status```
    
12) Fare il commit, descrivendo cosa si è fatto:  
    ```git commit -m "<descrizione delle modifiche effettuate>"```
    
13) Verificare di nuovo lo status (i file committati non dovrebbero apparire):  
    ```git status```
    
14) Caricare le modifiche su github (facendo login con nome e password):  
    ```git push```
    
15) Controllare su github.com che le modifiche ci siano effettivamente


## Branch

I branch ("rami") sono flussi di sviluppo del progetto che procedono in parallelo. Creare un nuovo branch è particolarmente comodo quando si vuole testare una nuova feature o sviluppare una nuova funzionalità, mantenendo intatto e disponibile il vecchio codice funzionante (su un branch parallelo). I commit effettuati su un branch sono del tutto indipendenti da ciò che avviene negli altri branch. Ad un certo punto, per esempio dopo che la feature è stata sviluppata e testata, si può voler ricongiungere il relativo branch a quello che l'ha generato: questa operazione è chiamata "merge".

Per visualizzare i branch esistenti (in verde quello in cui si trova attualmente):  
```git branch```

Per spostarsi su un altro branch:  
```git checkout <nome branch>```

Per creare e spostarsi su un nuovo branch:  
```git checkout -b <nome branch>```

Per fare il merge, occorre prima spostarsi sul branch padre:
```
git checkout <nome branch padre>  
git merge <nome branch da fondere>
```

## Merge conflict

I conflitti possono generarsi quando si tenta di unire i file di diverse sorgenti/versioni, per esempio durante un pull o un merge.
Per esempio, se esistono due versioni diverse di una porzione di codice e si prova ad unirle, git non può sapere a priori quale delle due preservare.
Di solito git è relativamente intelligente e cerca di generare meno conflitti possibili. Quando però succede, è l'utente che deve risolverli a mano.

La posizione in cui si è generato un conflict può essere trovata con `git status`.
A quel punto occorre aprire quel file con un editor di testo qualsiasi e localizzare il conflict.
In quella posizione, git avrà modificato il testo del file con il seguente pattern:  
```
<<<<<<< HEAD
<codice versione 1>
=======
<codice versione 2>
>>>>>>> <nome altro branch>
```
Adesso basta semplicemente cancellare il testo corrispondente alla versione sbagliata, inclusi i marker, lasciando solo il codice giusto. A quel punto fare un commit per risolvere definitivamente il merge conflict.
