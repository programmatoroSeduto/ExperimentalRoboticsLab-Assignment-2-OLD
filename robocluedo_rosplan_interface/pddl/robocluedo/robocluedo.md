# robocluedo -- notes about the PDDL 

## osservazioni iniziali

- il planner risolve il problema fino alla fine come se non ci fosse nessun problema
- è il sistema esterno a risolvere gli imprevisti con eventuali richieste di replanning



## system inizialization

the system shall be initialized, which means to allocate the hints and to update the ontology before starting. 

```
(init-planning-system )
	(*) check hint status
		(*) count hyotheses elements
		(*) classify hypothese into the three categories
	(*) check if the problem is solvable
		# is there at least one admissible hypothesis?
		(*) IF only one hypothesis at least active -> REPLAN
```

### check hint status

**assunzione** : dopo un tentativo di esecuzione di soluzione fallito, la knowledge base conserva ancora i dati derivati dall'ultima esecuzione. 

classificazione degli ID:

- un ID è *attivo* quando non si può provare che questi non è valido
- un ID è *escluso*  quando uno tra i campi *who*, *where* o *what* ha almeno due elementi associati
	questo è gestito dall'esterno: è il sistema esterno al PDDL che individua le esclusioni. il planner può escludere solo questi casi
- un ID è *ammissibile* quando i tre campi hanno uno e un solo elemento

il sistema iniziale valuta le condizioni delle ipotesi e marca i tre predicati col loro stato. 

il sistema fa anche il check se il problema è risolvibile. 

**il problema è risolvibile quando** esiste almeno un'opzione ancora attiva

**il problema è risolto per esclusione** quando esiste un'unica opzione valida o anche completa

**il problema non è risolvibile** quando tutte le opzioni sono state escluse



## Movimento

il robot si muove da un posto ad un altro. Ecco alcune osservazioni:

- non si può permanere su un posto per più di una mossa
- il posto indicato per il movimento non è il centro
- non essendoci ostacoli, il robot può muoversi in tutti i posti che vuole senza limiti
- ogni volta che il robot si muove, il valore di `(max-loop )` diminuisce di 1
- per muoversi, devono rimanere mosse fattibili al robot

```
(move-to ?from ?to)
```

### localizzazione e planning? 

**assunzione** : il movimento non fallisce ed è sempre completamente affidabile. quindi per la localizzazione basta solo leggere l'ultimo posto visitato. 



















```

```
