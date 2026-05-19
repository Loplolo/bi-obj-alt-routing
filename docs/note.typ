= Progetto di Ottimizzazione Su Grafo
#import "@preview/algorithmic:1.0.7"
#import algorithmic: algorithm, style-algorithm, algorithm-figure
#show: style-algorithm

== Introduzione
Il progetto consiste nell'implementazione e analisi delle euristiche ALT per la risoluzione di problemi Shortest-Path bi-obiettivo su digrafi, il caso analizzato utilizza il dataset del DIMACS 9 e compara diverse varianti dell'euristica ALT per ottimizzare la query del grafo avendo dei landmarks pre computati.

== graph_parse.cpp

Per leggere il contenuto dei file input la funzione `gzopen` si occupa di decomprimere i tre file contenenti le coordinate dei nodi, i valori distanza e tempo di percorrenza degli archi.
Il formato del file `.co` contenente le coordinate dei nodi consiste in stringhe divise per riga:
- Commenti che iniziano con il carattere `c`
- Definizione del numero di nodi e altre informazioni non rilevanti per l'implementazione `p aux sp co num_nodes`
- Associazione tra identificativo del nodo e coordinate x y con come formato `v id x y`
Similmente i file `.gr` contenenti le informazioni sugli archi:
- Commenti che iniziano con il carattere `c`
- Definizione del numero di nodi e archi `p sp num_nodes num_edges`
- Associazione tra l'id del nodo sorgente e target ed il peso dell'arco `a source target weight`
Per velocizzare la lettura viene utilizzato un buffer di dimensione massima spostato riga per riga che contiene tutti i caratteri un un array.
Per migliorare l'efficienza di memoria e la vicinanza nella cache, la struttura a digrafo che contiene queste informazioni ed è utilizzata nell'implementazione usa la rappresentazione Compressed Sparse Rows.
Il digrafo contiene all'interno di vettori lineari i valori interessati: coordinate, distanza e tempo di percorrenza; altri due vettori `offset` e `target` contengono invece la struttura del grafo. All'interno di `target` sono memorizzati sempre linearmente tutti i nodi di arrivo per ogni nodo sorgente, questo array contiene vicini localmente, con un offset variabile, tutti i nodi della stella uscente. Gli offset di ogni nodo sono memorizzati nel vettore `offset`, il quale associa ogni nodo `a` usato come indice la dimensione della sua stella uscente come `offset[a]`. 

Per esplorare la lista di adiacenza di un nodo viene utilizzato un ciclo nel range dell'offset:

#algorithm(
    line-numbers: false,
    {
    import algorithmic: *
    Assign($i$, $"offset"[u]$)
    For($i < "offset"[u + 1]$, {
        $"Print" "target"[i]$
      })
    })

Similmente i valori associati agli archi sono memorizzati linearmente con come indice `offset[u] + pos` con `pos` che rappresenta la locazione locale del arco nella stella uscente. La struttura mantiene anche il numero totale di archi e nodi.
Per rappresentare il digrafo inverso, utilizzato negli algoritmi bidirezionali, viene utilizzato un costruttore che copia il numero di nodi e archi del digrafo in avanti e utilizza gli indici `target` contando il numero di archi entranti, per poi utilizzare queste dimensioni per costruire il nuovo vettore `offset`. Infine riempie target e gli altri valori associandoli ai nodi corrispettivi. Precalcolare il grafo inverso permette di risparmiare tempo quando vengono chiamati gli algoritmi all'indietro, in questa implementazione avviene più di una volta.

== landmarks.cpp

I landmark sono memorizzati in una "struct of vectors" contenente i nodi selezionati, le distanze da e verso ogni nodo in vettori indicizzati da `landmark * num_nodes + node`.
Per calcolare i landmark viene implementato l'algoritmo di Dijkstra utilizzando strutture dati Radix bucket. L'accesso agli elementi dei bucket è implementato tramite una funzione inline che sfrutta l'aritmetica binaria con l'operatore xor `^` per ottenere la differenza tra i valori e la procedura built-in count leading zero `__builtin_clz()`; quest'ultima utilizza direttamente un'istruzione della CPU per calcolare l'indice del bucket in maniera efficiente. 

```cpp
inline int get_bucket(uint32_t val, uint32_t last) {
    if (val == last) return 0;
    uint32_t diff = val ^ last;
    return 32 - __builtin_clz(diff); // floor(log_2(diff)) + 1
}
```

Le politiche considerate per costruire l'insieme dei landmarks include `Random`, `Farthest`, `OptimizedFarthest`, `Avoid` e `MaxCover`. L'algoritmo ha come input un grafo che può essere in avanti o all'indietro e un numero che indica quale dei due valori di riferimento (distanza o tempo di percorrimento) considerare.
Per sfruttare la possibilità di parallelizzare le iterazioni di ricerca locale nella politica di `MaxCover` viene sfruttata la direttiva `#pragma omp parallel for schedule(dynamic)` che utilizza OpenMP per dividere le iterazioni del ciclo su più thread, `schedule(dynamic)` indica che le iterazioni sono assegnate ai thread dinamicamente appena si liberano invece che essere suddivisi preventivamente, siccome alcuni nodi possono avere più archi di altri il numero di iterazioni non è omogeneo e questa direttiva permette di utilizzare al massimo la CPU.

Per memorizzare e caricare i landmarks in memoria viene creato un file binario compresso tramite gzip contenente la struttura con all'interno i vettori che caratterizzano ogni landmark per una specifica metrica di valutazione.

== bi\_obj\_a\_star.cpp

L'implementazione segue l'algoritmo $"BOBA"^*$, utilizza sei chiamate alla procedura template di base che implementa $"A"^*$ con i metodi di early stopping dell'algoritmo. Viene chiamata due volte sul grafo in avanti e due volte sul grafo invertito, di queste una permuta l'ordine di comparazione dei valori in modo da individuare la soluzione paretiana.
Viene utilizzata una struttura `Node` per mantenere le etichette di ogni nodo vicine nella memoria. Questa struttura contiene il costo cumulato attuale dei due obiettivi `g1` e `g2`, la stima totale `g + h(v)` per ciascun obiettivo dentro a `f1` e `f2` e un riferimento al nodo precedente nel cammino `parent` per ricostruirlo facilmente. La funzione di ordinamento è implementata tramite l'overload dell'operatore `>` come ordinamento lessicografico.

La funzione ritorna un oggetto `Result` che contiene sia l'insieme delle soluzioni individuate `solutions` che la lista dei limiti inferiori da utilizzare negli step successivi, che equivale a `h'` nell'articolo.

Le euristiche `alt_h` e `alt_ub` rappresentano rispettivamente la disuguaglianza triangolare dei landmark e il limite superiore calcolato dal landmark, ovvero il valore ammissibile che non sovrastima $max_L |d(L,v)-d(v,L)| <= "dist"(u, v)$ e il valore che sovrastima ovver il minimo `d(u,L) + d(L,v) >= dist(u, v)`. Quest'ultimo è usato per aggiornare anticipatamente `g2_min[target]` in modo da eliminare diramazioni che non migliorano il fronte di Pareto.

La funzione template che implementa l'algoritmo permette come input sia `Graph` che `ReverseGraph` come l'implementazione di Dijkstra vista nella sezione (numero) e con euristiche passate come funzioni lambda per diminuire l'overhead grazie all'inlining del template. Il flag `inverted` scambia le colonne dei valori in modo da invertire la priorità nell'ordine lessicografico.

La funzione `query` esegue le 6 ricerche `boa` usando std::async per parallelizzare le prime 4 ricerche indipendenti, le quali individuano un limite inferiore per ogni nodo. Le ultime due ricerche sono anch'esse parallelizzate e utilizzano i limiti inferiori individuati dalle ricerche precedenti per rafforzare il pruning riducendo i nodi espansi. Usano come euristica il massimo tra l'eurisitica ALT standard `h` e il `lower_bound[u]` ottenuto negli step precedenti. Le ultime due ricerche avvengono sul grafo forward, ma permutando l'ordine dei valori di peso, per questo motivo il risultato della seconda deve invertire l'ordine dei valori, questo viene fatto in maniera efficiente con `std::swap`. 

L'implementazione segue l'algoritmo $"BOBA"^*$, utilizza sei chiamate alla procedura template di base che implementa $"A"^*$ con i metodi di early stopping dell'algoritmo. Viene chiamata due volte sul grafo in avanti e due volte sul grafo invertito; di queste, una permuta l'ordine di comparazione dei valori in modo da individuare la soluzione paretiana.

Viene utilizzata una struttura `Node` per mantenere le etichette di ogni nodo vicine nella memoria. Questa struttura contiene il costo cumulato attuale dei due obiettivi `g1` e `g2`, la stima totale `g + h(v)` per ciascun obiettivo dentro a `f1` e `f2` e un riferimento al nodo precedente nel cammino `parent` per ricostruirlo facilmente. La funzione di ordinamento è implementata tramite l'overload dell'operatore `>` come ordinamento lessicografico.

La funzione ritorna un oggetto `Result` che contiene sia l'insieme delle soluzioni individuate `solutions` che la lista dei limiti inferiori da utilizzare negli step successivi, che equivale a `h'` nell'articolo.

Le euristiche `alt_h` e `alt_ub` rappresentano rispettivamente la disuguaglianza triangolare dei landmark e il limite superiore calcolato dal landmark, ovvero il valore ammissibile che non sovrastima $max_L |d(L,v) - d(v,L)| <= "dist"(u, v)$ e il valore che sovrastima, ovvero il minimo $d(u,L) + d(L,v) >= "dist"(u, v)$. Quest'ultimo è usato per aggiornare anticipatamente `g2_min[target]` in modo da eliminare diramazioni che non migliorano il fronte di Pareto.

La funzione template che implementa l'algoritmo permette come input sia `Graph` che `ReverseGraph`, come l'implementazione di Dijkstra vista nella sezione (numero), e con euristiche passate come funzioni lambda per diminuire l'overhead grazie all'inlining del template. Il flag `inverted` scambia le colonne dei valori in modo da invertire la priorità nell'ordine lessicografico.

La funzione `query` esegue le 6 ricerche BOA usando `std::async` per parallelizzare le prime 4 ricerche indipendenti, le quali individuano un limite inferiore per ogni nodo. Le ultime due ricerche sono anch'esse parallelizzate e utilizzano i limiti inferiori individuati dalle ricerche precedenti per rafforzare il pruning, riducendo i nodi espansi. Usano come euristica il massimo tra l'euristica ALT standard `h` e il `lower_bound[u]` ottenuto negli step precedenti. Le ultime due ricerche avvengono sul grafo forward, ma permutando l'ordine dei valori di peso; per questo motivo, il risultato della seconda deve invertire l'ordine dei valori. Questo viene fatto in maniera efficiente con `std::swap`.

L'implementazione segue l'algoritmo $"BOBA"^*$, utilizza sei chiamate alla procedura template di base che implementa $"A"^*$ con i metodi di early stopping dell'algoritmo. Viene chiamata due volte sul grafo in avanti e due volte sul grafo invertito; di queste, una permuta l'ordine di comparazione dei valori in modo da individuare la soluzione paretiana.

Viene utilizzata una struttura `Node` per mantenere le etichette di ogni nodo vicine nella memoria. Questa struttura contiene il costo cumulato attuale dei due obiettivi `g1` e `g2`, la stima totale `g + h(v)` per ciascun obiettivo dentro a `f1` e `f2` e un riferimento al nodo precedente nel cammino `parent` per ricostruirlo facilmente. La funzione di ordinamento è implementata tramite l'overload dell'operatore `>` come ordinamento lessicografico.

La funzione ritorna un oggetto `Result` che contiene sia l'insieme delle soluzioni individuate `solutions` che la lista dei limiti inferiori da utilizzare negli step successivi, che equivale a `h'` nell'articolo.

Le euristiche `alt_h` e `alt_ub` rappresentano rispettivamente la disuguaglianza triangolare dei landmark e il limite superiore calcolato dal landmark, ovvero il valore ammissibile che non sovrastima $max_L |d(L,v) - d(v,L)| <= "dist"(u, v)$ e il valore che sovrastima, ovvero il minimo $d(u,L) + d(L,v) >= "dist"(u, v)$. Quest'ultimo è usato per aggiornare anticipatamente `g2_min[target]` in modo da eliminare diramazioni che non migliorano il fronte di Pareto.

La funzione template che implementa l'algoritmo permette come input sia `Graph` che `ReverseGraph`, come l'implementazione di Dijkstra vista nella sezione (numero), e con euristiche passate come funzioni lambda per diminuire l'overhead grazie all'inlining del template. Il flag `inverted` scambia le colonne dei valori in modo da invertire la priorità nell'ordine lessicografico.

La funzione `query` esegue le 6 ricerche BOA usando `std::async` per parallelizzare le prime 4 ricerche indipendenti, le quali individuano un limite inferiore per ogni nodo. Le ultime due ricerche sono anch'esse parallelizzate e utilizzano i limiti inferiori individuati dalle ricerche precedenti per rafforzare il pruning, riducendo i nodi espansi. Usano come euristica il massimo tra l'euristica ALT standard `h` e il `lower_bound[u]` ottenuto negli step precedenti. Le ultime due ricerche avvengono sul grafo forward, ma permutando l'ordine dei valori di peso; per questo motivo, il risultato della seconda deve invertire l'ordine dei valori. Questo viene fatto in maniera efficiente con `std::swap`.

== Complessità

- *Parsing* $O(n + m)$: Lettura lineare e costruzione del CSR con counting sort.
- *ReverseGraph* $O(n + m)$: Counting sort sugli archi invertiti.
- *Dijkstra Radix* $O(m + n log W)$: Utilizzando $W$ bit per i bucket, $n log W$ rappresenta le ridistribuzioni. Si ha una complessità ammortizzata quasi lineare in $m$.

- *Costruzione di $L$ landmark:*
  - *Random* $O(L (m + n log W))$: $L$ Dijkstra sui grafi in avanti e indietro.
  - *Farthest* $O(L^2 n + L (m + n log W))$: All'$i$-esima iterazione la selezione costa $O(n i)$; sommando da $0$ a $L-1$ si ottiene $O(L^2 n)$.
  - *Avoid* $O(L^2 n + L (m + n log W))$: Come *Farthest*; all'$i$-esima selezione viene chiamato Dijkstra ausiliario più un'analisi dell'albero ($n k = n i$).
  - *OptimizedFarthest* $O(P L (n L + m + n log W))$: $P$ è il numero di passi di ottimizzazione. Ogni passo fa $L$ iterazioni, salva/ripristina la riga del landmark ed esegue due volte Dijkstra, calcolando `farthest_score`. Il numero di passi $P$ non ha limiti teorici.
  - *MaxCover* $O(L (m + n k + n log W) + L^2 m k log L)$: La fase 1 costruisce $4L$ candidati utilizzando la politica *Avoid*, mentre la seconda fa una ricerca locale parallela. La seconda fase è dominante, con $log L$ iterazioni in cui ciascuna esegue una ricerca locale dove ogni candidato di scambio richiede `uncovered_arcs` = $O(m k)$. Con al massimo $O(L^2)$ coppie slot/candidato per iterazione, si ha $O(L^2 m k)$ per iterazione. Con $log L$ iterazioni e $T$ thread: $O((L^2 m k log L) / T)$.

- *Singola ricerca $"BOA"^*$* $O(|P| (m log(|P| n) + m k))$: Con $|P|$ la dimensione del fronte di Pareto. Nel caso peggiore ogni nodo può essere espanso più volte (una per ogni valore di `g2` che migliora `g2_min`), ma la dominanza garantisce che ciascun nodo sia espanso al più $|P|$ volte. Ogni espansione:
  - Estrae dall'heap: $O(log(|P| n))$.
  - Per ogni arco adiacente valuta l'euristica ALT: $O(k)$ con $k$ landmark.
  - Inserisce in coda: $O(log(|P| n))$.
- *Query* $O(|P| m k log(|P| n))$:
  - Round 1: 4 chiamate a BOA parallele $O(|P| m k log(|P| n))$.
  - Round 2: 2 chiamate a BOA parallele con $|P|$ effettivo inferiore $O(|P| m k log(|P| n))$.
  - Unione dei risultati: $O(|P| log |P|)$.

La complessità è dominata dalla fase di generazione dei landmark.
Per $k$ query si ha $O(n + m) + O(L (m + n log W)) + k O(|P| m k log(|P| n))$. Il collo di bottiglia sta nella dimensione di $|P|$, che viene ridotto grazie all'aggiornamento anticipato e all'euristica ALT.

== Risultati

// Numero di landmarks: 0, 2, 4, 8, 16
// Policy: None, Random, Farthest, OptimizedFarthest, Avoid, MaxCover
// Totale 30 combinazioni

// Asse x numero di landmarks e asse y il tempo
// colore diverso per ogni policy

