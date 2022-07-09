# robocluedo -- notes about the PDDL 

## system inizialization

the system shall be initialized, which means to allocate the hints and to update the ontology before starting. 

```
(init-planning-system )
	(*) check hint status
```

### check hint status

**assunzione** : dopo un tentativo di esecuzione di soluzione fallito, la knowledge base conserva ancora i dati derivati dall'ultima esecuzione. 

classificazione degli ID:

- un ID è *attivo* quando non si può provare che questi non è valido
- un ID è *escluso*  quando uno tra i campi *who*, *where* o *what* ha almeno due elementi associati
	questo è gestito dall'esterno: è il sistema esterno al PDDL che individua le esclusioni. il planner può escludere solo questi casi
- un ID è *ammissibile* quando i tre campi hanno uno e un solo elemento























```

```
